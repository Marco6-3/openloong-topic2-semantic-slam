#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/runtime_env.sh"
exec rosrun openloong_semantic_slam runtime_monitor.py
