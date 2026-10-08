#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${1:-/home/wjkxy/ros2_ws}"
OUT_DIR="/tmp/fkie_husky_spawn_graph"
MASTER_PORT="${MASTER_PORT:-11348}"
MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
GZ_LOG="${OUT_DIR}/gzserver_graph.log"

mkdir -p "${OUT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export GAZEBO_MASTER_URI="${MASTER_URI}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"

echo "=== Gazebo ROS spawn service graph debug ==="
echo "WS_DIR=${WS_DIR}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"
echo

echo "[1/7] Check Gazebo ROS packages and factory plugin"
ros2 pkg prefix gazebo_ros
ros2 pkg prefix gazebo_msgs
echo "--- factory plugin path ---"
ls -l /opt/ros/humble/lib/libgazebo_ros_factory.so || true
echo "--- factory plugin missing shared libs ---"
ldd /opt/ros/humble/lib/libgazebo_ros_factory.so | grep -i "not found" || echo "[OK] no missing shared libs reported"
echo

echo "[2/7] Force-clean old Gazebo/spawn processes and ROS2 daemon cache"
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f spawn_entity.py 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
sleep 3

echo "[3/7] Start gzserver with factory plugin on isolated Gazebo master"
rm -f "${GZ_LOG}"
gzserver /usr/share/gazebo-11/worlds/empty.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_force_system.so \
  --verbose > "${GZ_LOG}" 2>&1 &
GZ_PID=$!
echo "gzserver_pid=${GZ_PID}"
sleep 5

if ! ps -p "${GZ_PID}" >/dev/null 2>&1; then
  echo "[ERROR] gzserver exited early"
  tail -n 200 "${GZ_LOG}" || true
  exit 2
fi

echo "[4/7] CLI service graph after daemon reset"
echo "--- ros2 service list -t | grep spawn ---"
ros2 service list -t | grep -E "spawn|gazebo" || true
echo "--- ros2 service type /spawn_entity ---"
ros2 service type /spawn_entity || true
echo "--- ros2 node list ---"
ros2 node list | grep -E "gazebo|spawn" || true
echo

echo "[5/7] Direct rclpy graph probe for gazebo_msgs/srv/SpawnEntity"
python3 - <<'PY'
import time

import rclpy
from gazebo_msgs.srv import SpawnEntity

rclpy.init()
node = rclpy.create_node('spawn_entity_graph_probe')
client = node.create_client(SpawnEntity, '/spawn_entity')

for i in range(20):
    services = node.get_service_names_and_types()
    interesting = [
        (name, types)
        for name, types in services
        if 'spawn' in name or 'gazebo' in name
    ]
    print(f'--- graph probe {i + 1}/20 ---')
    for name, types in interesting:
        print(f'{name}: {types}')
    if client.wait_for_service(timeout_sec=1.0):
        print('[OK] /spawn_entity is available with gazebo_msgs/srv/SpawnEntity')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    time.sleep(1.0)

print('[FAIL] /spawn_entity was not available as gazebo_msgs/srv/SpawnEntity')
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY
PROBE_RC=$?

echo "[6/7] Try minimal spawn only if typed service is available"
if [[ "${PROBE_RC}" -eq 0 ]]; then
  MIN_URDF="${OUT_DIR}/minimal_box.urdf"
  SPAWN_LOG="${OUT_DIR}/spawn_box.log"
  cat > "${MIN_URDF}" <<'EOF'
<?xml version="1.0"?>
<robot name="minimal_box">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual><geometry><box size="0.5 0.5 0.5"/></geometry></visual>
    <collision><geometry><box size="0.5 0.5 0.5"/></geometry></collision>
  </link>
</robot>
EOF
  set +e
  timeout 30s ros2 run gazebo_ros spawn_entity.py \
    -entity minimal_box \
    -file "${MIN_URDF}" \
    -timeout 20 \
    -x 0 -y 0 -z 1 > "${SPAWN_LOG}" 2>&1
  SPAWN_RC=$?
  set -e
  echo "--- spawn rc=${SPAWN_RC} ---"
  cat "${SPAWN_LOG}" || true
else
  echo "[SKIP] typed /spawn_entity service is not available"
fi

echo "[7/7] gzserver log tail"
tail -n 220 "${GZ_LOG}" || true

echo
echo "Cleanup:"
echo "  pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f spawn_entity.py"

exit "${PROBE_RC}"
