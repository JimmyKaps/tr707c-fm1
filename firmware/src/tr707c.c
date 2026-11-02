/* SPDX-License-Identifier: GPL-3.0-only */
/* TR-707C: a drum machine for the M-VAVE FM-1, after the look and the workflow of a classic 1985 rhythm composer.
 * Built on Dinghy (Felucca / SLOOP platform, GPL-3.0): see LICENSING.md. An unofficial fan firmware: it is not made,
 * endorsed or supported by Roland or M-VAVE.
 *
 *   13 voices       BD SD LT MT HT RIM CLAP COW CH OH CRASH RIDE TAMB (synthesised, integers only)
 *   sequencer       every instrument has its own pattern of up to 64 bars (1024 steps) and its own loop length 1..64
 *   STEP mode       the 16 white keys are the 16 steps of the bar on the screen (tap: off > on > accent);
 *                   the black keys play the selected instrument
 *   PAD mode        keys 1..13 (F3..F4) play the 13 instruments; REC writes them into the pattern while it runs
 *   encoders        SELECT instrument | ALGORITHM bar | PRESETS loop length | KNOB1 tempo | KNOB2 swing
 *                   KNOB3 level (decay on page 2) | KNOB4 tune (accent on page 2)
 *   buttons         PLAY start/stop | REC arm | SEL step/pad mode | FX mute | SEQ clear (press twice) | ARP copy bar > next
 *                   EDIT knob page | LFO clock INT/EXT | GLO clock out | SAVE / HOME save / load song | OCT-/+ song 1..8
 *   MIDI            see the table in docs/TR-707C.md: notes 36..48 (ch 10: General MIDI drums), 3 CC banks of 13,
 *                   globals, clock in/out, start/stop. Made for a controller like the Arturia KeyStep.
 *
 * Integers only (no FPU), no 64-bit division: the tables come from tools/gen_tr707c_tables.py.
 * What platform.c offers: docs/API.md. The chip's rules: docs/RULES.md. */
#define APP_NAME "TR-707C"
#define STORE_MAGIC 0x43373037u                 /* "707C": songs are not read by any other firmware, nor theirs by this one */
static void app_idle(void);
#define LCD_IDLE app_idle                       /* while the screen's DMA runs: MIDI in (platform.c) */
#include "platform.c"
#include "tr707c_tables.h"

#define NV 13u                                  /* instruments */
#define SPB 16u                                 /* steps per bar */
#define MAXB 64u                                /* bars per instrument */
#define TSTEPS (MAXB * SPB)                     /* 1024 steps */
enum { V_BD, V_SD, V_LT, V_MT, V_HT, V_RIM, V_CLP, V_COW, V_CH, V_OH, V_CR, V_RD, V_TB };
static const char *const NAME[NV] = {"BASS DRUM", "SNARE DRUM", "LOW TOM", "MID TOM", "HI TOM", "RIM SHOT", "HAND CLAP",
                                     "COWBELL", "CLOSED HAT", "OPEN HAT", "CRASH", "RIDE", "TAMBOURINE"};
static const char *const SHORT[NV] = {"BD", "SD", "LT", "MT", "HT", "RM", "CP", "CB", "CH", "OH", "CR", "RD", "TB"};
static const uint8_t GM[NV] = {36, 38, 41, 45, 48, 37, 39, 56, 42, 46, 49, 51, 54};

/* ---- the song: what is saved (one blob of 4 KB: 8 songs) and what the sound ISR reads */
typedef struct { uint8_t cells[TSTEPS / 4u]; uint8_t len, mute, p[3]; } trk_t;   /* 2 bits a step: 0 off 1 on 2 accent */
typedef struct { uint16_t bpm10; uint8_t swing, acc, ver, pad[3]; trk_t t[NV]; } song_t;      /* 3397 B */
#define SONG_VER 1u
static song_t song;
static int step_get(const trk_t *t, uint32_t i) { return t->cells[i >> 2] >> ((i & 3u) << 1) & 3u; }
static void step_set(trk_t *t, uint32_t i, uint32_t v)
{
    uint32_t sh = (i & 3u) << 1;
    t->cells[i >> 2] = (uint8_t)((t->cells[i >> 2] & ~(3u << sh)) | (v & 3u) << sh);
}

/* ---- shared between the audio ISR and the main loop (one writer each) */
static volatile uint32_t run, ext_clk, step_ctr, into, cur_len, sps_even, sps_odd, clk_thr, clk_out;
static volatile uint32_t rq_start, rq_stop, ext_n;          /* main loop counts up; the ISR follows */
static volatile uint16_t lq[16];                            /* live hits, main > ISR: voice | vel << 4 | accent << 11 */
static volatile uint32_t lq_w, lq_r;
static volatile uint16_t evq[64];                           /* what sounded, ISR > main: voice | vel << 4, 0x8000 = clock */
static volatile uint32_t evq_w, evq_r;
static volatile int32_t mast = 200;                         /* Q8 (CC 7) */

/* ---- the voices (ISR only) */
typedef struct {
    int32_t env, lp, gain, pe;                  /* envelope Q24, noise filter, gain Q8, pitch envelope Q15 */
    uint32_t m4, ph1, ph2, inc1, inc2, sweep, hp, cnt, age;
    uint32_t active;
} vst_t;
static vst_t vs[NV];
static uint32_t rng = 0x2545F491u;

static int32_t sine(uint32_t ph)                /* a full cycle over 2^32 -> -32767..32767 (as dinghy.c) */
{
    int32_t x = (int32_t)ph >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15;
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}

static void evq_put(uint32_t e) { evq[evq_w & 63u] = (uint16_t)e; evq_w++; }

static void voice_hit(uint32_t v, uint32_t vel, uint32_t accent)
{
    const tr_vdef_t *d = &TR_VDEF[v];
    const trk_t *t = &song.t[v];
    vst_t *s = &vs[v];
    uint32_t tune = TR_TUNE[t->p[1] & 127u], g;
    if (t->mute || vel == 0u) return;
    if (v == V_CH) vs[V_OH].active = 0;         /* the closed hat chokes the open one */
    s->inc1 = (uint32_t)((uint64_t)d->inc0 * tune >> 16);
    s->inc2 = (uint32_t)((uint64_t)s->inc1 * d->ratio >> 8);
    s->sweep = (uint32_t)((uint64_t)d->sweep * tune >> 16);
    s->hp = d->hp;
    if (!d->inc0) {                             /* no pitch (clap, tambourine): tune moves the noise filter instead */
        uint32_t h = d->hp * tune >> 16;
        s->hp = h < 4u ? 4u : h > 250u ? 250u : h;
    }
    s->m4 = TR_DEC4[v][t->p[2] & 127u];
    g = (vel * 2u * ((uint32_t)t->p[0] * 2u)) >> 8;
    if (accent) g = g * (256u + (uint32_t)song.acc * 2u) >> 8;
    s->gain = (int32_t)g;
    s->env = 1 << 24;
    s->pe = 32767;
    s->ph1 = s->ph2 = 0;
    s->lp = 0;
    s->cnt = s->age = 0;
    s->active = 1;
    evq_put(v | vel << 4);
}

static void render(int32_t *out, uint32_t n)    /* add n frames of every sounding voice to out (L R L R ...) */
{
    uint32_t v, i;
    for (v = 0; v < NV; v++) {
        vst_t *s = &vs[v];
        const tr_vdef_t *d = &TR_VDEF[v];
        uint32_t ph1, ph2, inc, cnt, np = d->noise;
        int32_t env, lp, gain, pe;
        if (!s->active) continue;
        ph1 = s->ph1; ph2 = s->ph2; cnt = s->cnt; env = s->env; lp = s->lp; gain = s->gain; pe = s->pe;
        inc = s->inc1 + (uint32_t)((uint64_t)s->sweep * (uint32_t)pe >> 15);
        for (i = 0; i < n; i++) {
            int32_t tone = 0, nz, x;
            if (!(cnt++ & 3u)) {                /* every 4 frames: envelopes */
                if (v == V_CLP && s->age < 330u && s->age % 110u == 0u) env = 1 << 24;   /* three claps */
                s->age++;
                env = (int32_t)((int64_t)env * s->m4 >> 24);
                if (d->sweep) {
                    pe -= pe >> d->pdec;
                    inc = s->inc1 + (uint32_t)((uint64_t)s->sweep * (uint32_t)pe >> 15);
                }
                if (env < 16384) { s->active = 0; break; }
            }
            if (s->inc1) {
                ph1 += inc;
                if (d->metal) { ph2 += s->inc2; tone = ((int32_t)ph1 < 0 ? 9000 : -9000) + ((int32_t)ph2 < 0 ? 9000 : -9000); }
                else tone = sine(ph1) * 3 >> 2;
            }
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            nz = (int32_t)(rng >> 16) - 32768;
            lp += ((nz - lp) * (int32_t)s->hp) >> 8;            /* noise minus its low-pass = high-passed noise */
            x = (tone * (int32_t)(255u - np) + (nz - lp) * (int32_t)np) >> 8;
            x = (x * (env >> 9)) >> 15;
            x = (x * gain) >> 8;
            x = (x * mast) >> 9;                /* ~0.4 at the default master: room for a few voices at once */
            out[2u * i] += x;
            out[2u * i + 1u] += x;
        }
        s->ph1 = ph1; s->ph2 = ph2; s->cnt = cnt; s->env = env; s->lp = lp; s->pe = pe;
    }
}

/* the sequencer: every instrument reads its pattern at its own position */
static void do_step(void)
{
    uint32_t v, k = step_ctr;
    for (v = 0; v < NV; v++) {
        const trk_t *t = &song.t[v];
        uint32_t len = (uint32_t)t->len * SPB, pos = len ? k % len : 0u, c = (uint32_t)step_get(t, pos);
        if (c) voice_hit(v, c == 2u ? 127u : 96u, c == 2u);
    }
    cur_len = (k & 1u) ? sps_odd : sps_even;
    into = 0;
    step_ctr = k + 1u;
}

static uint32_t clk_into, ext_seen, ext_cnt, rs_seen, rp_seen;
static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t done = 0;
    while (rq_start != rs_seen) { rs_seen++; step_ctr = 0; into = cur_len = 1u; ext_cnt = 5u; clk_into = 0; run = 1; }
    while (rq_stop != rp_seen) { rp_seen++; run = 0; }
    while (lq_r != lq_w) {                      /* live hits from the keys and from MIDI */
        uint32_t e = lq[lq_r & 15u];
        lq_r++;
        voice_hit(e & 15u, e >> 4 & 127u, e >> 11 & 1u);
    }
    while (ext_seen != ext_n) {                 /* external clock: 24 pulses a quarter, a step every 6 */
        ext_seen++;
        if (run && ext_clk && ++ext_cnt >= 6u) { ext_cnt = 0; do_step(); }
    }
    while (done < n) {
        uint32_t seg = n - done;
        int internal = run && !ext_clk;
        if (internal) {
            if (into >= cur_len) { do_step(); continue; }
            if (seg > cur_len - into) seg = cur_len - into;
        }
        render(out + 2u * done, seg);
        done += seg;
        if (internal) {
            into += seg;
            if (clk_out) {                      /* MIDI clock out: 6 a step */
                clk_into += seg;
                while (clk_into >= clk_thr && clk_thr) { clk_into -= clk_thr; evq_put(0x8000u); }
            }
        }
    }
}

/* ---- main loop from here: tempo, patterns, MIDI, screen */
static struct { uint8_t sel, mode, page, rec, slot, bar, clr; } ui;   /* mode 0 STEP, 1 PAD; bar = the bar on the screen */
static uint32_t idle_ok, hit_t[NV], param_t, save_req, load_req, clr_t;
static char param_txt[24];
static uint8_t WK[16];                          /* key index of the 16 white keys (F3 ..) */
#define KEY0 53u
static int black(uint32_t note) { return (0x54Au >> (note % 12u)) & 1u; }

static void tempo_apply(void)
{
    uint32_t b = song.bpm10 < 400u ? 400u : song.bpm10 > 2400u ? 2400u : song.bpm10;
    uint32_t sps = 6615000u / b, sw = 50u + (uint32_t)song.swing * 25u / 127u;
    song.bpm10 = (uint16_t)b;
    sps_even = sps * 2u * sw / 100u;
    sps_odd = sps * (200u - 2u * sw) / 100u;
    clk_thr = sps / 6u;
}

static void put_hit(uint32_t v, uint32_t vel, uint32_t accent)   /* main > ISR */
{
    lq[lq_w & 15u] = (uint16_t)(v | vel << 4 | accent << 11);
    lq_w++;
}

static void record_hit(uint32_t v, uint32_t vel)                 /* write a live hit at the nearest step */
{
    trk_t *t = &song.t[v];
    uint32_t len = (uint32_t)t->len * SPB, k = step_ctr, i = into, l = cur_len;
    if (!run || !k || !len) return;
    k = (k - 1u) % len;                         /* the step that played last */
    if (i * 2u > l) k = (k + 1u) % len;         /* past the middle of it: the next one */
    step_set(t, k, vel >= 110u ? 2u : 1u);
}

static void live(uint32_t v, uint32_t vel)
{
    if (v >= NV) return;
    put_hit(v, vel, vel >= 110u);
    if (ui.rec) record_hit(v, vel);
}

static void param_say(const char *a, uint32_t val)
{
    char *p = param_txt;
    while (*a) *p++ = *a++;
    *p++ = ' ';
    if (val >= 100u) *p++ = (char)('0' + val / 100u);
    if (val >= 10u) *p++ = (char)('0' + val / 10u % 10u);
    *p++ = (char)('0' + val % 10u);
    *p = 0;
    param_t = fm1_ms;
}

static void demo_song(void)
{
    uint32_t b, k, i;
    trk_t *t;
    memset(&song, 0, sizeof song);
    song.ver = SONG_VER; song.bpm10 = 1200; song.acc = 64;
    for (i = 0; i < NV; i++) { song.t[i].len = 1; song.t[i].p[0] = 100; song.t[i].p[1] = 64; song.t[i].p[2] = 64; }
    t = &song.t[V_BD]; t->len = 8;
    for (b = 0; b < 8; b++) for (k = 0; k < 4; k++) step_set(t, b * 16 + k * 4, k ? 1u : 2u);
    step_set(t, 7 * 16 + 14, 1); step_set(t, 3 * 16 + 10, 1);
    t = &song.t[V_SD]; t->len = 4;
    for (b = 0; b < 4; b++) { step_set(t, b * 16 + 4, 2); step_set(t, b * 16 + 12, 2); }
    step_set(t, 3 * 16 + 15, 1); step_set(t, 3 * 16 + 14, 1);
    t = &song.t[V_CH]; t->len = 4;
    for (i = 0; i < 64; i += 2) step_set(t, i, i % 4u ? 1u : 2u);
    t = &song.t[V_OH]; t->len = 4; step_set(t, 6, 1); step_set(t, 22, 1); step_set(t, 38, 1); step_set(t, 54, 1);
    t = &song.t[V_CLP]; t->len = 8; step_set(t, 20, 1); step_set(t, 52, 1); step_set(t, 36, 1); step_set(t, 116, 1);
    t = &song.t[V_RIM]; t->len = 2; step_set(t, 3, 1); step_set(t, 7, 1); step_set(t, 11, 1); step_set(t, 13, 1); step_set(t, 19, 1); step_set(t, 26, 1);
    t = &song.t[V_COW]; t->len = 3; step_set(t, 10, 1); step_set(t, 27, 1); step_set(t, 43, 1);
    t = &song.t[V_TB]; t->len = 2; for (i = 2; i < 20; i += 4) step_set(t, i, 1); step_set(t, 30, 1);
    t = &song.t[V_LT]; t->len = 4; step_set(t, 56, 1); step_set(t, 59, 1);
    t = &song.t[V_MT]; t->len = 4; step_set(t, 60, 1); step_set(t, 62, 1);
    t = &song.t[V_HT]; t->len = 4; step_set(t, 61, 1); step_set(t, 63, 1);
    t = &song.t[V_CR]; t->len = 8; step_set(t, 0, 1); t->p[0] = 70;
    t = &song.t[V_RD]; t->len = 8; for (i = 0; i < 128; i += 3) step_set(t, i, 1); t->p[0] = 60;
    song.t[V_CH].p[0] = 85; song.t[V_TB].p[0] = 75; song.t[V_COW].p[0] = 80;
}

static int song_ok(void)
{
    uint32_t i;
    if (song.ver != SONG_VER) return 0;
    for (i = 0; i < NV; i++) if (song.t[i].len < 1u || song.t[i].len > MAXB || song.t[i].p[0] > 127u || song.t[i].p[1] > 127u || song.t[i].p[2] > 127u) return 0;
    return 1;
}
static int quiet(void)
{
    uint32_t v;
    for (v = 0; v < NV; v++) if (vs[v].active) return 0;
    return 1;
}

/* ---- MIDI */
static int note_voice(uint32_t ch, uint32_t note)
{
    uint32_t v;
    if (ch == 9u) {                             /* channel 10: General MIDI drums */
        for (v = 0; v < NV; v++) if (GM[v] == note) return (int)v;
        return -1;
    }
    return note >= 36u && note < 36u + NV ? (int)(note - 36u) : -1;   /* else: 13 keys from C1 */
}

static void midi_cc(uint32_t cc, uint32_t val)
{
    trk_t *t = &song.t[ui.sel];
    if (cc >= 20u && cc <= 32u) song.t[cc - 20u].p[0] = (uint8_t)val;        /* bank A: level */
    else if (cc >= 40u && cc <= 52u) song.t[cc - 40u].p[1] = (uint8_t)val;   /* bank B: tune */
    else if (cc >= 60u && cc <= 72u) song.t[cc - 60u].p[2] = (uint8_t)val;   /* bank C: decay */
    else switch (cc) {
    case 1:   song.swing = (uint8_t)val; tempo_apply(); break;
    case 7:   mast = (int32_t)val * 2; break;
    case 14:  song.bpm10 = (uint16_t)(400u + val * 2000u / 127u); tempo_apply(); break;
    case 15:  song.acc = (uint8_t)val; break;
    case 80:  ui.sel = (uint8_t)(val * NV / 128u); break;
    case 81:  ui.bar = (uint8_t)(val >> 1); break;
    case 82:  t->len = (uint8_t)(1u + (val >> 1)); break;
    case 83:  ui.rec = val >= 64u; break;
    case 84:  if (val >= 64u) memset(t->cells, 0, sizeof t->cells); break;
    case 85:  t->mute = val >= 64u; break;
    case 86:  ext_clk = val >= 64u; break;
    case 120: case 123: { uint32_t v; for (v = 0; v < NV; v++) vs[v].active = 0; } break;   /* all sound / notes off */
    }
}

static void midi_in(void)
{
    uint8_t st, d1, d2;
    int src;
    while ((src = midi_get(&st, &d1, &d2)) != 0) {
        uint32_t ty = st & 0xF0u;
        if (st == 0xF8u) { ext_n++; continue; }
        if (st == 0xFAu) { rq_start++; continue; }
        if (st == 0xFBu) continue;                              /* CONTINUE: no song position, ignored */
        if (st == 0xFCu) { rq_stop++; continue; }
        if (ty == 0x90u && d2) {
            int v = note_voice(st & 15u, d1);
            if (v >= 0) { ui.sel = (uint8_t)v; live((uint32_t)v, d2); }
        } else if (ty == 0xB0u) midi_cc(d1, d2);
    }
}
static void app_idle(void) { if (idle_ok) midi_in(); }

/* ---- the screen: a beige case, orange and black lettering, a green LCD */
#define C_CASE RGB(206, 202, 190)
#define C_DARK RGB(150, 146, 134)
#define C_KEYC RGB(224, 220, 208)
#define C_ORG RGB(228, 87, 30)
#define C_ORGL RGB(238, 170, 140)
#define C_INK RGB(27, 26, 24)
#define C_LCD RGB(174, 189, 152)
#define C_LCDI RGB(38, 48, 31)
#define C_LCDD RGB(150, 166, 130)
#define C_REC RGB(200, 30, 20)

static char *ps(char *p, const char *s) { while (*s) *p++ = *s++; return p; }
static char *pu(char *p, uint32_t v, uint32_t digits)
{
    uint32_t d, div = 1;
    for (d = 1; d < digits; d++) div *= 10u;
    for (; div; div /= 10u) *p++ = (char)('0' + v / div % 10u);
    return p;
}

static void draw_head(void)
{
    char s[32];
    uint32_t x;
    cv_begin(240, 64, C_CASE);
    cv_rect(0, 0, 240, 4, C_ORG);
    cv_rect(0, 4, 240, 4, C_INK);
    x = cv_text(10, 12, &FONT_L, "TR-", C_INK);
    cv_text((int32_t)x, 12, &FONT_L, "707C", C_ORG);
    ps(s, "COMPUTER RHYTHM COMPOSER");
    s[24] = 0;
    cv_text(10, 46, &FONT_S, s, C_INK);
    cv_blit(0, 0);
}

typedef struct { uint8_t sel, bar, len, mute, play, view; uint32_t lo, hi, bpm; char par[24]; } lcd_t;
static lcd_t lcd_now, lcd_old;
static int lcd_same(void)
{
    const char *a = (const char *)&lcd_now, *b = (const char *)&lcd_old;
    uint32_t i;
    for (i = 0; i < sizeof lcd_now; i++) if (a[i] != b[i]) return 0;
    return 1;
}
static void draw_lcd(void)
{
    char s[24], *p;
    uint32_t i, w;
    cv_begin(240, 64, C_CASE);
    cv_rect(6, 0, 228, 62, C_INK);
    cv_rect(9, 3, 222, 56, C_LCD);
    cv_text(14, 3, &FONT_L, NAME[lcd_now.sel], lcd_now.mute ? C_LCDD : C_LCDI);
    p = ps(s, "BAR "); p = pu(p, lcd_now.view + 1u, 2u); p = ps(p, "/64"); *p = 0;
    cv_text(14, 36, &FONT_S, s, C_LCDI);
    p = ps(s, "LOOP "); p = pu(p, lcd_now.len, 2u); *p = 0;
    cv_text(104, 36, &FONT_S, s, C_LCDI);
    p = pu(s, lcd_now.bpm / 10u, 3u); *p++ = '.'; *p++ = (char)('0' + lcd_now.bpm % 10u); *p = 0;
    w = text_w(&FONT_S, s);
    cv_text(226 - (int32_t)w, 36, &FONT_S, s, C_LCDI);
    for (i = 0; i < 64u; i++) {                 /* the 64 bars: has notes, the one on screen, the one playing */
        uint32_t has = i < 32u ? lcd_now.lo >> i & 1u : lcd_now.hi >> (i - 32u) & 1u;
        int32_t x = 16 + (int32_t)i * 3;
        cv_rect(x, 54, 2, 3, i >= lcd_now.len ? C_LCDD : has ? C_LCDI : C_LCD);
        if (i >= lcd_now.len || !has) cv_rect(x, 56, 2, 1, i >= lcd_now.len ? C_LCDD : C_LCDD);
        if (i == lcd_now.view) cv_rect(x - 1, 52, 4, 1, C_LCDI);
        if (i == lcd_now.play) cv_rect(x, 51, 2, 2, C_ORG);
    }
    cv_blit(0, 64);
}

static uint32_t steps_old = ~0u, steps_pos_old, flash_old = ~0u, mutes_old = ~0u;
static uint8_t sel_old = 255, rec_old = 255;
static char info_old[40], status_old[40];
static int same(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return *a == *b; }
static void copy(char *d, const char *s) { while ((*d++ = *s++) != 0) ; }
static void draw_steps(uint32_t bits, uint32_t playpos, const char *info)
{
    uint32_t i;
    cv_begin(240, 56, C_CASE);
    for (i = 0; i < 16u; i++) {
        uint32_t c = bits >> (i * 2u) & 3u;
        int32_t x = 8 + (int32_t)i * 14;
        cv_rect(x, 2, 13, 32, c ? C_ORG : i % 4u == 0u ? C_ORGL : C_KEYC);
        if (c == 2u) cv_rect(x, 2, 13, 5, C_INK);
        if (i == playpos) cv_rect(x, 36, 13, 3, C_INK);
    }
    cv_text(8, 40, &FONT_S, info, C_INK);
    cv_blit(0, 128);
}

static void draw_voices(uint32_t flash, uint32_t mutes, const char *status, int recon)
{
    uint32_t i;
    cv_begin(240, 56, C_CASE);
    for (i = 0; i < NV; i++) {                  /* two rows: 7 and 6 */
        uint32_t row = i >= 7u ? 1u : 0u, col = i - row * 7u;
        int32_t x = 8 + (int32_t)col * 32, y = (int32_t)row * 20;
        uint32_t sel = i == ui.sel, hit = flash >> i & 1u;
        cv_rect(x, y, 30, 18, sel ? C_INK : mutes >> i & 1u ? C_DARK : C_KEYC);
        cv_rect(x, y, 30, 2, hit || sel ? C_ORG : C_DARK);
        cv_text(x + 6, y + 2, &FONT_S, SHORT[i], sel ? C_ORG : C_INK);
    }
    cv_text(8, 38, &FONT_S, status, recon ? C_REC : C_INK);
    cv_rect(0, 54, 240, 1, C_ORG);
    cv_rect(0, 55, 240, 1, C_INK);
    cv_blit(0, 184);
}

static void save_song(void)
{
    if (blob_save(ui.slot, &song, sizeof song) == 0) param_say("SAVED SONG", (uint32_t)ui.slot + 1u);
    else param_say("SAVE FAILED", 0);
}
static void load_song(void)
{
    static song_t tmp;
    if (blob_load(ui.slot, &tmp, sizeof tmp) == 0 && tmp.ver == SONG_VER) {
        memcpy(&song, &tmp, sizeof song);
        if (!song_ok()) demo_song();
        tempo_apply();
        param_say("LOADED SONG", (uint32_t)ui.slot + 1u);
    } else
        param_say("EMPTY SONG", (uint32_t)ui.slot + 1u);
}

static void app_main(void)
{
    uint32_t prev = 0, k, w = 0;
    for (k = 0; k < 27u; k++) if (!black(KEY0 + k) && w < 16u) WK[w++] = (uint8_t)k;
    demo_song();
    ui.sel = 0; ui.slot = 0;
    if (blob_load(0, &song, sizeof song) || !song_ok()) demo_song();     /* song 1, if one was saved */
    tempo_apply();
    cur_len = sps_even;
    lcd_fill(0, 0, 240, 240, C_CASE);
    draw_head();

    for (;;) {
        uint32_t keys, down, up, rel, b, v, i, bar_play = 99u, steppos = 99u;
        int32_t e;
        trk_t *t = &song.t[ui.sel];
        platform_poll();

        keys = fm1_in.notes;
        down = keys & ~prev;
        up = prev & ~keys;
        (void)up;
        prev = keys;
        if (ui.mode == 0u) {                    /* STEP: white keys are the steps, black keys play the instrument */
            for (k = 0; k < 16u; k++)
                if (down >> WK[k] & 1u) {
                    uint32_t idx = (uint32_t)ui.bar * SPB + k, c = ((uint32_t)step_get(t, idx) + 1u) % 3u;
                    step_set(t, idx, c);
                    if (c) put_hit(ui.sel, c == 2u ? 127u : 96u, c == 2u);       /* hear what you set */
                }
            for (k = 0; k < 27u; k++)
                if ((down >> k & 1u) && black(KEY0 + k)) live(ui.sel, 100u);
        } else {                                /* PAD: F3.. = the 13 instruments */
            for (k = 0; k < NV; k++)
                if (down >> k & 1u) { ui.sel = (uint8_t)k; live(k, 100u); }
        }

        b = fm1_input_edges(&rel);
        if (b & 1u << BTN_PLAY) { if (run) rq_stop++; else rq_start++; if (clk_out && !ext_clk) midi_send(run ? 0xFCu : 0xFAu, 0, 0); }
        if (b & 1u << BTN_REC) ui.rec = !ui.rec;
        if (b & 1u << BTN_SEL) ui.mode = !ui.mode;
        if (b & 1u << BTN_FX) t->mute = !t->mute;
        if (b & 1u << BTN_EDIT) ui.page = !ui.page;
        if (b & 1u << BTN_LFO) ext_clk = !ext_clk;
        if (b & 1u << BTN_GLO) clk_out = !clk_out;
        if (b & 1u << BTN_OCT_DOWN && ui.slot > 0u) { ui.slot--; param_say("SONG", (uint32_t)ui.slot + 1u); }
        if (b & 1u << BTN_OCT_UP && ui.slot < 7u) { ui.slot++; param_say("SONG", (uint32_t)ui.slot + 1u); }
        if (b & 1u << BTN_SAVE) { save_req = 1; if (run) rq_stop++; }
        if (b & 1u << BTN_HOME) { load_req = 1; if (run) rq_stop++; }
        if (b & 1u << BTN_ARP) {                /* copy this bar to the next */
            uint32_t a = (uint32_t)ui.bar * SPB, d = ((uint32_t)(ui.bar + 1u) % MAXB) * SPB;
            for (i = 0; i < SPB; i++) step_set(t, d + i, (uint32_t)step_get(t, a + i));
            ui.bar = (uint8_t)((ui.bar + 1u) % MAXB);
            if (t->len <= ui.bar) t->len = (uint8_t)(ui.bar + 1u);
        }
        if (b & 1u << BTN_SEQ) {                /* clear: twice within 1.5 s */
            if (ui.clr && fm1_ms - clr_t < 1500u) { memset(t->cells, 0, sizeof t->cells); ui.clr = 0; param_say("CLEARED", 0); }
            else { ui.clr = 1; clr_t = fm1_ms; }
        }
        if (ui.clr && fm1_ms - clr_t >= 1500u) ui.clr = 0;

        if ((e = fm1_enc_take(ENC_SELECT)) != 0) { int32_t s = (int32_t)ui.sel + e; ui.sel = (uint8_t)(s < 0 ? 0 : s > 12 ? 12 : s); }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) { int32_t s = (int32_t)ui.bar + e; ui.bar = (uint8_t)(s < 0 ? 0 : s > 63 ? 63 : s); }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) { int32_t s = (int32_t)t->len + e; t->len = (uint8_t)(s < 1 ? 1 : s > 64 ? 64 : s); param_say("LOOP BARS", t->len); }
        if ((e = fm1_enc_take(ENC_KNOB1)) != 0) { int32_t s = (int32_t)song.bpm10 + e * 10; song.bpm10 = (uint16_t)(s < 400 ? 400 : s > 2400 ? 2400 : s); tempo_apply(); param_say("TEMPO", song.bpm10 / 10u); }
        if ((e = fm1_enc_take(ENC_KNOB2)) != 0) { int32_t s = (int32_t)song.swing + e * 4; song.swing = (uint8_t)(s < 0 ? 0 : s > 127 ? 127 : s); tempo_apply(); param_say("SWING", 50u + song.swing * 25u / 127u); }
        if ((e = fm1_enc_take(ENC_KNOB3)) != 0) {
            uint8_t *q = &t->p[ui.page ? 2 : 0];
            int32_t s = (int32_t)*q + e * 4; *q = (uint8_t)(s < 0 ? 0 : s > 127 ? 127 : s);
            param_say(ui.page ? "DECAY" : "LEVEL", *q);
        }
        if ((e = fm1_enc_take(ENC_KNOB4)) != 0) {
            if (ui.page) { int32_t s = (int32_t)song.acc + e * 4; song.acc = (uint8_t)(s < 0 ? 0 : s > 127 ? 127 : s); param_say("ACCENT", song.acc); }
            else { int32_t s = (int32_t)t->p[1] + e * 2; t->p[1] = (uint8_t)(s < 0 ? 0 : s > 127 ? 127 : s); param_say("TUNE", t->p[1]); }
        }

        midi_in();
        if (save_req && !run && quiet()) { save_song(); save_req = 0; }      /* an erase stalls the sound: save when silent */
        if (load_req && !run && quiet()) { load_song(); load_req = 0; }

        while (evq_r != evq_w) {                /* what the ISR played: lights and MIDI out */
            uint32_t ev = evq[evq_r & 63u];
            evq_r++;
            if (ev & 0x8000u) { if (clk_out) midi_send(0xF8u, 0, 0); }
            else {
                v = ev & 15u;
                if (v < NV) {
                    hit_t[v] = fm1_ms;
                    midi_send(0x99u, GM[v], ev >> 4 & 127u);
                    midi_send(0x89u, GM[v], 0);
                }
            }
        }

        /* ---- what the screen and the LEDs show */
        {
            uint32_t pos = 0, len = (uint32_t)t->len * SPB, sc = step_ctr;
            if (run && sc && len) {
                pos = (sc - 1u) % len;
                bar_play = pos / SPB;
                if (bar_play == ui.bar) steppos = pos % SPB;
            }
        }
        {
            uint32_t flash = 0, mutes = 0, bits = 0, lo = 0, hi = 0;
            char s[40], *p;
            for (k = 0; k < NV; k++) {
                if (fm1_ms - hit_t[k] < 90u) flash |= 1u << k;
                if (song.t[k].mute) mutes |= 1u << k;
            }
            for (i = 0; i < SPB; i++) bits |= (uint32_t)step_get(t, (uint32_t)ui.bar * SPB + i) << (i * 2u);
            for (i = 0; i < MAXB; i++) {
                const uint8_t *c = &t->cells[i * 4u];
                if (c[0] | c[1] | c[2] | c[3]) { if (i < 32u) lo |= 1u << i; else hi |= 1u << (i - 32u); }
            }
            /* LEDs: STEP mode shows the steps on the white keys; PAD mode lights what sounded */
            for (k = 0; k < 27u; k++) fm1_led_key(KEY_LED(k), 0);
            if (ui.mode == 0u) {
                for (i = 0; i < 16u; i++) {
                    uint32_t c = bits >> (i * 2u) & 3u;
                    fm1_led_key(KEY_LED(WK[i]), c == 2u || (i == steppos));
                    fm1_led_key_glow(KEY_LED(WK[i]), c == 1u);
                }
            } else
                for (k = 0; k < NV; k++) { fm1_led_key(KEY_LED(k), flash >> k & 1u); fm1_led_key_glow(KEY_LED(k), k == ui.sel); }
            fm1_led_key(BTN_PLAY, run != 0u);
            fm1_led_key(BTN_REC, ui.rec);
            fm1_led_key(BTN_SEL, ui.mode);
            fm1_led_key(BTN_FX, t->mute);
            fm1_led_key(BTN_EDIT, ui.page);
            fm1_led_key(BTN_LFO, ext_clk != 0u);
            fm1_led_key(BTN_GLO, clk_out != 0u);

            idle_ok = 1;
            lcd_now.sel = ui.sel; lcd_now.view = ui.bar; lcd_now.len = t->len; lcd_now.mute = t->mute;
            lcd_now.play = (uint8_t)(bar_play < 64u ? bar_play : 99u); lcd_now.lo = lo; lcd_now.hi = hi; lcd_now.bpm = song.bpm10;
            if (!lcd_same()) { lcd_old = lcd_now; draw_lcd(); }
            p = s;
            if (param_t && fm1_ms - param_t < 2000u) p = ps(p, param_txt);
            else if (ui.clr) p = ps(p, "CLEAR? PRESS SEQ AGAIN");
            else p = ps(p, ui.page ? "K3 DECAY K4 ACCENT" : "K3 LEVEL K4 TUNE");
            *p = 0;
            if (bits != steps_old || steppos != steps_pos_old || !same(s, info_old)) {
                steps_old = bits; steps_pos_old = steppos; copy(info_old, s);
                draw_steps(bits, steppos, s);
            }
            p = s;
            p = ps(p, ui.mode ? "PAD " : "STEP ");
            p = ps(p, run ? "PLAY " : "STOP ");
            if (ui.rec) p = ps(p, "REC ");
            p = ps(p, ext_clk ? "EXT " : "INT ");
            p = ps(p, "S"); p = pu(p, (uint32_t)ui.slot + 1u, 1u);
            *p = 0;
            if (flash != flash_old || mutes != mutes_old || ui.sel != sel_old || ui.rec != rec_old || !same(s, status_old)) {
                flash_old = flash; mutes_old = mutes; sel_old = ui.sel; rec_old = ui.rec; copy(status_old, s);
                draw_voices(flash, mutes, s, ui.rec);
            }
            idle_ok = 0;
        }
    }
}
