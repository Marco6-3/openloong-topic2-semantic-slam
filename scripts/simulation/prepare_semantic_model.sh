#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
model_dir="$repo_root/data/models"
export_state="$repo_root/data/simulation/model-export"
source_model="$model_dir/yolov8n-seg.pt"
onnx_model="$model_dir/yolov8n-seg-320.onnx"
source_url="https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n-seg.pt"
source_sha="a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c"
known_onnx_sha="a603aedac954586f458c1341817169a80d26b11683dae10dfaab230004e3de47"

mkdir -p "$model_dir" "$export_state"
if [[ -f "$onnx_model" ]] && echo "$known_onnx_sha  $onnx_model" | sha256sum --check --status; then
  echo "[model] verified cached $(basename "$onnx_model")"
  exit 0
fi

if [[ ! -f "$source_model" ]] || ! echo "$source_sha  $source_model" | sha256sum --check --status; then
  curl -fL --retry 5 --retry-delay 2 --progress-bar "$source_url" -o "$source_model.part"
  echo "$source_sha  $source_model.part" | sha256sum --check --status
  mv "$source_model.part" "$source_model"
fi

export YOLO_CONFIG_DIR="$export_state"
pixi exec \
  --channel conda-forge \
  --spec "python=3.12.*" \
  --spec "pytorch=2.9.1=cpu_mkl_py312*" \
  --spec "torchvision=0.24.1=*cpu*" \
  --spec "ultralytics=8.4.132" \
  --spec "onnx=1.22.0" \
  python "$repo_root/scripts/simulation/export_semantic_model.py" "$source_model" "$onnx_model"

actual_sha="$(sha256sum "$onnx_model" | awk '{print $1}')"
echo "[model] ONNX SHA-256: $actual_sha"
