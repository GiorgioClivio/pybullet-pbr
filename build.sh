#!/usr/bin/env bash
# Builds a local pybullet extension from ./bullet into ./build. Never installs anything.
set -euo pipefail
cd "$(dirname "$0")"
ROOT=$PWD
PY=${PYTHON:-python3}

say() { echo "[pybullet-pbr] $*"; }

$PY -c 'import sys' 2>/dev/null || { echo "no usable python: $PY" >&2; exit 1; }
say "Python: $($PY -c 'import sys;print(".".join(map(str,sys.version_info[:3])))')"
INC=$($PY -c 'import sysconfig;print(sysconfig.get_paths()["include"])')
[ -f "$INC/Python.h" ] || { echo "missing Python headers ($INC/Python.h) - install python3-dev" >&2; exit 1; }
say "Compiler: $(${CC:-gcc} --version | head -1)"
[ -f /usr/include/GL/gl.h ] || { echo "missing GL headers - install libgl1-mesa-dev" >&2; exit 1; }
[ -f /usr/include/X11/Xlib.h ] || { echo "missing X11 headers - install libx11-dev" >&2; exit 1; }
say "OpenGL: available"
$PY -c 'import setuptools' 2>/dev/null || { echo "setuptools missing in $PY" >&2; exit 1; }

if [ ! -d bullet ]; then
  command -v git >/dev/null || { echo "git is needed to fetch the Bullet source" >&2; exit 1; }
  say "Fetching Bullet source (shallow clone, ~500 MB)..."
  git clone --depth 1 https://github.com/bulletphysics/bullet3.git bullet || exit 1
fi

say "Applying the integration patch..."
$PY patches/apply.py || exit 1

# GCC 15 promotes -Wincompatible-pointer-types to an error; pybullet.c passes
# PyObject* to numpy's PyArray_DATA(PyArrayObject*). Benign, so demote it back.
export CFLAGS="${CFLAGS:-} -Wno-incompatible-pointer-types"

say "Building local renderer..."
cd bullet
set +e
$PY setup.py build_ext --build-lib "$ROOT/build" "$@" > "$ROOT/build.log" 2>&1
status=$?
set -e
if [ $status -ne 0 ]; then
  grep -E "error:|error;|Error " "$ROOT/build.log" | head -20 >&2
  echo "build failed (exit $status) - full log in build.log" >&2
  exit 1
fi
SO=$(ls "$ROOT"/build/pybullet*.so 2>/dev/null | head -1) || true
[ -n "${SO:-}" ] || { echo "build produced no extension - see build.log" >&2; exit 1; }
say "Built:"
echo "./build/$(basename "$SO")"
