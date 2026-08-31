#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
source "$script_dir/runtime_env.sh"

duration="${1:-60}"
output="${2:-$repo_root/data/outputs/simulation/runtime-validation.json}"
exec python "$script_dir/validate_runtime.py" --duration "$duration" --output "$output"
