#!/usr/bin/env bash
set -e
fuel_run_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$fuel_run_root/env.sh"
exec /usr/bin/python3 "$fuel_run_root/tools/bringup.py" "$@"
