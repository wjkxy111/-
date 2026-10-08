#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${1:-/home/wjkxy/ros2_ws}"
PKG_DIR="${WS_DIR}/src/fkie_husky_manipulation_simulation"
PKG_INSTALL="${WS_DIR}/install/fkie_husky_manipulation_simulation/share/fkie_husky_manipulation_simulation"
WORLD="${PKG_INSTALL}/worlds/blueberry_greenhouse.world"
XACRO_FILE="${PKG_INSTALL}/urdf/husky_panda.urdf.xacro"
URDF_FILE="/tmp/husky_panda_ros2_debug.urdf"
GZ_LOG="/tmp/fkie_husky_gzserver_debug.log"
SPAWN_LOG="/tmp/fkie_husky_spawn_entity_debug.log"
MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export GAZEBO_MASTER_URI="${MASTER_URI}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="${PKG_DIR}/models:${PKG_INSTALL}/models:${HOME}/.gazebo/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${PKG_INSTALL}:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

echo "=== FKIE Husky Panda spawn debug ==="
echo "WS_DIR=${WS_DIR}"
echo "WORLD=${WORLD}"
echo "XACRO_FILE=${XACRO_FILE}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"
echo

if [[ ! -f "${WORLD}" ]]; then
  echo "[ERROR] World file not found: ${WORLD}" >&2
  exit 1
fi

if [[ ! -f "${XACRO_FILE}" ]]; then
  echo "[ERROR] Xacro file not found: ${XACRO_FILE}" >&2
  exit 1
fi

echo "[1/7] Cleaning stale Gazebo processes"
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 2

echo "[2/7] Generating URDF"
xacro "${XACRO_FILE}" > "${URDF_FILE}"
grep -n "<robot name=" "${URDF_FILE}" || true
grep -n "<plugin" "${URDF_FILE}" || true

echo "[3/7] Starting gzserver without GUI"
rm -f "${GZ_LOG}" "${SPAWN_LOG}"
gzserver "${WORLD}" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_force_system.so \
  --verbose > "${GZ_LOG}" 2>&1 &
GZSERVER_PID=$!
echo "gzserver pid: ${GZSERVER_PID}"

echo "[4/7] Waiting for /spawn_entity"
FOUND_SPAWN=0
for _ in $(seq 1 120); do
  if ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then
    FOUND_SPAWN=1
    break
  fi
  if ! ps -p "${GZSERVER_PID}" >/dev/null 2>&1; then
    echo "[ERROR] gzserver exited before /spawn_entity appeared" >&2
    tail -n 160 "${GZ_LOG}" || true
    exit 2
  fi
  sleep 1
done

if [[ "${FOUND_SPAWN}" != "1" ]]; then
  echo "[ERROR] /spawn_entity did not appear within 120 seconds" >&2
  echo "--- gzserver log tail ---"
  tail -n 200 "${GZ_LOG}" || true
  exit 3
fi

echo "[OK] /spawn_entity is available"
ros2 service list | grep spawn || true

echo "[5/7] Spawning original Husky + Panda from URDF file"
set +e
ros2 run gazebo_ros spawn_entity.py \
  -entity husky \
  -file "${URDF_FILE}" \
  -timeout 120 \
  -x -1.5 \
  -y 0.0 \
  -z 0.35 \
  -Y 0.0 > "${SPAWN_LOG}" 2>&1
SPAWN_RC=$?
set -e

cat "${SPAWN_LOG}"
echo "spawn return code: ${SPAWN_RC}"

echo "[6/7] Current ROS/Gazebo topics"
echo "--- ROS topics ---"
ros2 topic list | grep -E "robot_description|tf|joint_states|scan|laser|camera|image|depth|odom|clock" || true
echo "--- Gazebo topics ---"
gz topic -l 2>/dev/null | grep -E "husky|laser|realsense|camera|world|pose|model" || true

echo "[7/7] Log tails"
echo "--- gzserver log tail: ${GZ_LOG} ---"
tail -n 220 "${GZ_LOG}" || true

echo "--- spawn log: ${SPAWN_LOG} ---"
cat "${SPAWN_LOG}" || true

if [[ "${SPAWN_RC}" -eq 0 ]]; then
  echo
  echo "[OK] Spawn completed in headless gzserver."
  echo "To open GUI against this server, run in another terminal:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
  echo "  gzclient --verbose"
else
  echo
  echo "[ERROR] Spawn failed or timed out. Please paste this script output."
fi

exit "${SPAWN_RC}"
