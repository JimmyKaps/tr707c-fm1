#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Run firmware/src/tr707c.c on this computer (tests/host.c): build/host/tr707c.wav and the screens as PNG.
#   tests/run_host_tr707c.sh [script]      (script lines: see tests/host.c; tests/tr707c_demo.txt is the demo)
set -e
cd "$(dirname "$0")/.."
mkdir -p build/host
python3 tools/gen_tr707c_tables.py >/dev/null
cc -O2 -Wall -Wno-unused-function -DDINGHY_HOST -DAPP_VER="\"${APP_VER:-dev}\"" -Ifirmware/hal -Ifirmware/src -Ifirmware/gen -o build/host/tr707c firmware/src/tr707c.c
build/host/tr707c build/host/tr707c ${1:+"$1"}
python3 - <<'PY'
from pathlib import Path
from PIL import Image
for p in sorted(Path("build/host").glob("tr707c-*.ppm")):
    Image.open(p).resize((480, 480), Image.NEAREST).save(p.with_suffix(".png"))
    p.unlink()
    print("screen ->", p.with_suffix(".png"))
PY
