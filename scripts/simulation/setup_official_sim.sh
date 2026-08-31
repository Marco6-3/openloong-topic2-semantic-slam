#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
data_root="$repo_root/data/simulation"
download_dir="$data_root/downloads"
official_dir="$data_root/official"
workspace="$data_root/catkin_ws"
patch_file="$repo_root/simulation/patches/official-frame-fix.patch"
stamp_file="$workspace/src/.openloong-official-stamp"
model_url="https://openloong.obs.cn-east-3.myhuaweicloud.com/public/static/contests/master-1/gazebo_models.zip"
source_url="https://openloong.obs.cn-east-3.myhuaweicloud.com/public/static/contests/master-1/src.zip"
model_sha="676f2c0034e31a7dc1024086b56c9bd71690c46c68448a39c0af18df54fdabe0"
source_sha="0b518cf584967c664aa45554c6031fff3c9074ed479cecc74d1aebe8a6cc9d99"

mkdir -p "$download_dir" "$official_dir" "$workspace/src"

fetch_verified() {
  local url="$1"
  local destination="$2"
  local expected_sha="$3"
  if [[ -f "$destination" ]] && echo "$expected_sha  $destination" | sha256sum --check --status; then
    echo "[setup] verified cached $(basename "$destination")"
    return
  fi
  curl -fL --retry 3 --progress-bar "$url" -o "$destination.part"
  echo "$expected_sha  $destination.part" | sha256sum --check --status
  mv "$destination.part" "$destination"
  echo "[setup] downloaded and verified $(basename "$destination")"
}

check_archive_paths() {
  local archive="$1"
  unzip -t "$archive" >/dev/null
  local unsafe
  unsafe="$(unzip -Z1 "$archive" | awk '$0 ~ /^\// || $0 ~ /(^|\/)\.\.($|\/)/ {print; exit}')"
  if [[ -n "$unsafe" ]]; then
    echo "unsafe ZIP path in $archive: $unsafe" >&2
    exit 1
  fi
}

fetch_verified "$source_url" "$download_dir/src.zip" "$source_sha"
fetch_verified "$model_url" "$download_dir/gazebo_models.zip" "$model_sha"
check_archive_paths "$download_dir/src.zip"
check_archive_paths "$download_dir/gazebo_models.zip"

unzip -q -o "$download_dir/src.zip" -d "$official_dir"
unzip -q -o "$download_dir/gazebo_models.zip" -d "$official_dir"

patch_sha="$(sha256sum "$patch_file" | awk '{print $1}')"
expected_stamp="$source_sha $patch_sha"
current_stamp="$(cat "$stamp_file" 2>/dev/null || true)"
if [[ "$current_stamp" != "$expected_stamp" ]] \
  || [[ ! -d "$workspace/src/vehicle_simulator" ]] \
  || [[ ! -d "$workspace/src/velodyne_simulator" ]]; then
  # Recreate only the generated package copies; the tracked custom package is linked below.
  rm -rf "$workspace/src/vehicle_simulator" "$workspace/src/velodyne_simulator"
  cp -a "$official_dir/src/vehicle_simulator" "$workspace/src/vehicle_simulator"
  cp -a "$official_dir/src/velodyne_simulator" "$workspace/src/velodyne_simulator"
  chmod +x "$workspace/src/vehicle_simulator/scripts/wait_for_gazebo_models.sh"
  patch --batch -d "$workspace/src" -p0 < "$patch_file"
  printf '%s\n' "$expected_stamp" > "$stamp_file"
  echo "[setup] refreshed patched official source"
else
  echo "[setup] patched official source is current"
fi
[[ -n "${CONDA_PREFIX:-}" ]] || { echo "setup must run inside the simulation Pixi environment" >&2; exit 1; }
rm -f "$workspace/src/CMakeLists.txt"
ln -s "$CONDA_PREFIX/share/catkin/cmake/toplevel.cmake" "$workspace/src/CMakeLists.txt"
ln -sfn "$repo_root/simulation/src/openloong_semantic_slam" "$workspace/src/openloong_semantic_slam"

echo "[setup] official simulation prepared under $data_root"
