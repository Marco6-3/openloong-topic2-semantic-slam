#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="$repo_root/data/simulation/catkin_ws"

if [[ ! -f "$workspace/src/CMakeLists.txt" ]]; then
  echo "simulation workspace is missing; run the setup task first" >&2
  exit 1
fi

# Recover from an interrupted/invalid first configure (the official ZIP ships an empty top-level CMakeLists).
if [[ -f "$workspace/build/CMakeCache.txt" && ! -d "$workspace/build/catkin_generated" ]]; then
  rm -rf "$workspace/build" "$workspace/devel"
fi

cd "$workspace"
catkin_make -DCMAKE_BUILD_TYPE=Release
echo "[build] simulation workspace built at $workspace"
