#!/bin/bash
# Run the booster_mujoco simulator
# Usage: ./run_sim.sh [extra args]  (e.g. -i 1 -n lo -r t1)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export BOOSTER_FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/fastdds_profile.xml"

exec "${SCRIPT_DIR}/simulate/build/booster_mujoco" "$@"
