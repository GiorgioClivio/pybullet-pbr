#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
rm -rf build build.log bullet/build
echo "[pybullet-pbr] cleaned (bullet source kept)"
