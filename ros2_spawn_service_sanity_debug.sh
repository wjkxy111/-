#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${1:-/home/wjkxy/ros2_ws}"
OUT_DIR="/tmp/fkie_husky_spawn_sanity"
MASTER_PORT="${MASTER_PORT:-11347}"
MASTER_URI="http://127.0.0.1:${MASTER_PORT}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

mkdir -p "${OUT_DIR}"

export GAZEBO_MASTER_URI="${MASTER_URI}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"

MIN_URDF="${OUT_DIR}/minimal_box.urdf"
GZ_LOG="${OUT_DIR}/gzserver_minimal.log"
SPAWN_LOG="${OUT_DIR}/spawn_minimal.log"

cat > "${MIN_URDF}" <<'EOF'
<?xml version="1.0"?>
<robot name="minimal_box">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
  </link>
</robot>
EOF

echo "=== spawn service sanity debug ==="
echo "WS_DIR=${WS_DIR}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "MIN_URDF=${MIN_URDF}"
echo

echo "[1/5] Force-clean Gazebo processes"
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f spawn_entity.py 2>/dev/null || true
sleep 3

echo "[2/5] Start empty gzserver on isolated port ${MASTER_PORT}"
rm -f "${GZ_LOG}" "${SPAWN_LOG}"
gzserver /usr/share/gazebo-11/worlds/empty.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_force_system.so \
  --verbose > "${GZ_LOG}" 2>&1 &
GZ_PID=$!
echo "gzserver_pid=${GZ_PID}"

echo "[3/5] Wait for /spawn_entity"
for i in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -qx "/spawn_entity"; then
    echo "[OK] /spawn_entity appeared after ${i}s"
    break
  fi
  if ! ps -p "${GZ_PID}" >/dev/null 2>&1; then
    echo "[ERROR] gzserver exited"
    tail -n 160 "${GZ_LOG}" || true
    exit 2
  fi
  sleep 1
done

ros2 service list | grep spawn || true

echo "[4/5] Spawn minimal box through gazebo_ros spawn_entity.py"
set +e
timeout 30s ros2 run gazebo_ros spawn_entity.py \
  -entity minimal_box \
  -file "${MIN_URDF}" \
  -timeout 20 \
  -x 0 \
  -y 0 \
  -z 1 > "${SPAWN_LOG}" 2>&1
RC=$?
set -e

echo "--- spawn rc=${RC} ---"
cat "${SPAWN_LOG}" || true

echo "[5/5] Inspect Gazebo state"
echo "--- gz model topics ---"
gz topic -l 2>/dev/null | grep -E "minimal_box|model|pose" || true
echo "--- gzserver log tail ---"
tail -n 180 "${GZ_LOG}" || true

if [[ "${RC}" -eq 0 ]]; then
  echo "[PASS] Minimal spawn service works. The Husky URDF is the remaining problem."
else
  echo "[FAIL] Minimal spawn service did not return. This is a Gazebo/ROS service runtime problem, not the Husky model yet."
fi

echo
echo "Cleanup command:"
echo "  pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f spawn_entity.py"

exit "${RC}"
