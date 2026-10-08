#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${1:-/home/wjkxy/ros2_ws}"
PKG_SRC="${WS_DIR}/src/fkie_husky_manipulation_simulation"
PKG_INSTALL="${WS_DIR}/install/fkie_husky_manipulation_simulation/share/fkie_husky_manipulation_simulation"
WORLD="${PKG_INSTALL}/worlds/blueberry_greenhouse.world"
EMPTY_WORLD="${PKG_INSTALL}/worlds/empty.world"
XACRO_FILE="${PKG_INSTALL}/urdf/husky_panda.urdf.xacro"
OUT_DIR="/tmp/fkie_husky_spawn_matrix"
MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

mkdir -p "${OUT_DIR}"

export GAZEBO_MASTER_URI="${MASTER_URI}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="${PKG_SRC}/models:${PKG_INSTALL}/models:${HOME}/.gazebo/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${PKG_INSTALL}:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

FULL_URDF="${OUT_DIR}/husky_panda_full.urdf"
NO_PLUGIN_URDF="${OUT_DIR}/husky_panda_no_plugins.urdf"
NO_GAZEBO_URDF="${OUT_DIR}/husky_panda_no_gazebo_blocks.urdf"

echo "=== spawn matrix debug ==="
echo "WS_DIR=${WS_DIR}"
echo "WORLD=${WORLD}"
echo "EMPTY_WORLD=${EMPTY_WORLD}"
echo "XACRO_FILE=${XACRO_FILE}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "OUT_DIR=${OUT_DIR}"
echo

echo "[0] Clean stale processes/nodes"
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f spawn_entity.py 2>/dev/null || true
sleep 2

echo "[1] Generate URDF variants"
xacro "${XACRO_FILE}" > "${FULL_URDF}"

python3 - "${FULL_URDF}" "${NO_PLUGIN_URDF}" "${NO_GAZEBO_URDF}" <<'PY'
import re
import sys
from pathlib import Path

src = Path(sys.argv[1])
no_plugin = Path(sys.argv[2])
no_gazebo = Path(sys.argv[3])

text = src.read_text()

# Remove all Gazebo plugin elements but leave links, joints, transmissions,
# Gazebo material/friction blocks, and sensor definitions intact.
without_plugins = re.sub(
    r'\s*<plugin\b[^>]*>.*?</plugin>',
    '',
    text,
    flags=re.DOTALL,
)
no_plugin.write_text(without_plugins)

# Remove every Gazebo extension block. This is only a temporary diagnostic
# URDF to prove whether the visual/collision model itself can be inserted.
without_gazebo = re.sub(
    r'\s*<gazebo(?:\s+reference="[^"]*")?>.*?</gazebo>',
    '',
    text,
    flags=re.DOTALL,
)
no_gazebo.write_text(without_gazebo)

for label, value in [
    ("full_plugins", len(re.findall(r'<plugin\b', text))),
    ("no_plugin_plugins", len(re.findall(r'<plugin\b', without_plugins))),
    ("full_gazebo_blocks", len(re.findall(r'<gazebo(?:\s+reference="[^"]*")?>', text))),
    ("no_gazebo_blocks", len(re.findall(r'<gazebo(?:\s+reference="[^"]*")?>', without_gazebo))),
]:
    print(f"{label}={value}")
PY

echo "--- plugin lines in full URDF ---"
grep -n "<plugin\|libgazebo\|realsense_gazebo\|ros_control\|ray_sensor\|gpu_laser" "${FULL_URDF}" || true
echo

run_case() {
  local case_name="$1"
  local world_file="$2"
  local urdf_file="$3"
  local gz_log="${OUT_DIR}/${case_name}_gzserver.log"
  local spawn_log="${OUT_DIR}/${case_name}_spawn.log"

  echo
  echo "=== CASE ${case_name} ==="
  echo "world=${world_file}"
  echo "urdf=${urdf_file}"

  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
  pkill -f spawn_entity.py 2>/dev/null || true
  sleep 2

  rm -f "${gz_log}" "${spawn_log}"
  gzserver "${world_file}" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so \
    -s libgazebo_ros_force_system.so \
    --verbose > "${gz_log}" 2>&1 &
  local gz_pid=$!
  echo "gzserver_pid=${gz_pid}"

  local found=0
  for _ in $(seq 1 90); do
    if ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then
      found=1
      break
    fi
    if ! ps -p "${gz_pid}" >/dev/null 2>&1; then
      echo "[FAIL] gzserver exited before /spawn_entity"
      tail -n 160 "${gz_log}" || true
      return 20
    fi
    sleep 1
  done

  if [[ "${found}" != "1" ]]; then
    echo "[FAIL] /spawn_entity did not appear"
    tail -n 160 "${gz_log}" || true
    return 21
  fi

  echo "[OK] /spawn_entity available"
  set +e
  timeout 75s ros2 run gazebo_ros spawn_entity.py \
    -entity "husky_${case_name}" \
    -file "${urdf_file}" \
    -timeout 60 \
    -x -1.5 \
    -y 0.0 \
    -z 0.35 \
    -Y 0.0 > "${spawn_log}" 2>&1
  local rc=$?
  set -e

  echo "--- spawn rc=${rc} ---"
  cat "${spawn_log}" || true
  echo "--- gazebo model/entity topics ---"
  gz topic -l 2>/dev/null | grep -E "husky|model|pose|laser|camera|realsense" || true
  echo "--- gzserver log tail ---"
  tail -n 180 "${gz_log}" || true

  if [[ "${rc}" -eq 0 ]]; then
    echo "[PASS] ${case_name}"
  elif [[ "${rc}" -eq 124 ]]; then
    echo "[TIMEOUT] ${case_name}: spawn_entity did not return within 75s"
  else
    echo "[FAIL] ${case_name}: spawn_entity rc=${rc}"
  fi

  pkill -f gzserver 2>/dev/null || true
  pkill -f spawn_entity.py 2>/dev/null || true
  sleep 2
  return 0
}

run_case "empty_no_gazebo" "${EMPTY_WORLD}" "${NO_GAZEBO_URDF}" || true
run_case "empty_no_plugins" "${EMPTY_WORLD}" "${NO_PLUGIN_URDF}" || true
run_case "empty_full" "${EMPTY_WORLD}" "${FULL_URDF}" || true
run_case "greenhouse_no_gazebo" "${WORLD}" "${NO_GAZEBO_URDF}" || true
run_case "greenhouse_no_plugins" "${WORLD}" "${NO_PLUGIN_URDF}" || true
run_case "greenhouse_full" "${WORLD}" "${FULL_URDF}" || true

echo
echo "=== summary files ==="
ls -lh "${OUT_DIR}"
echo
echo "Please paste the output from this script, especially the first failing case."
