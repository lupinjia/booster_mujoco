#!/bin/bash
# DDS round-trip test for booster_mujoco:
#   - subscribes rt/low_state
#   - publishes rt/joint_ctrl (tau=1 on all joints)
# Usage: ./run_test.sh [network_interface]
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export BOOSTER_FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/fastdds_profile.xml"

IFACE="${1:-lo}"
exec "${SCRIPT_DIR}/simulate/build/test" "${IFACE}"
