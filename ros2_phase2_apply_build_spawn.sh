#!/usr/bin/env bash
set -eo pipefail

WS_DIR="${1:-/home/wjkxy/ros2_ws}"
PKG_NAME="fkie_husky_manipulation_simulation"
PKG_SRC="${WS_DIR}/src/${PKG_NAME}"
PKG_INSTALL="${WS_DIR}/install/${PKG_NAME}/share/${PKG_NAME}"
STAMP="$(date +%Y%m%d_%H%M%S)"
MASTER_PORT="${MASTER_PORT:-11349}"
MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
OUT_DIR="/tmp/fkie_husky_phase2_spawn"
WORLD_KIND="${WORLD_KIND:-greenhouse}"

mkdir -p "${OUT_DIR}"

if [[ ! -d "${PKG_SRC}/urdf" ]]; then
  echo "[ERROR] Package source not found: ${PKG_SRC}" >&2
  exit 1
fi

echo "=== ROS2 phase2 apply/build/spawn ==="
echo "WS_DIR=${WS_DIR}"
echo "PKG_SRC=${PKG_SRC}"
echo "MASTER_URI=${MASTER_URI}"
echo "WORLD_KIND=${WORLD_KIND}"
echo

echo "[1/9] Apply reversible ROS1 Gazebo plugin isolation patch to source xacros"
python3 - "${PKG_SRC}" "${STAMP}" <<'PY'
import re
import sys
from pathlib import Path

pkg_dir = Path(sys.argv[1])
stamp = sys.argv[2]
urdf_dir = pkg_dir / "urdf"

gpu_laser_re = re.compile(
    r'<plugin\s+filename="libgazebo_ros_gpu_laser\.so"\s+name="([^"]*)">\s*'
    r'<topicName>(.*?)</topicName>\s*'
    r'<frameName>(.*?)</frameName>\s*'
    r'(?:<robotNamespace>.*?</robotNamespace>\s*)?'
    r'</plugin>',
    re.DOTALL,
)

control_re = re.compile(
    r'<plugin\s+filename="libgazebo_ros_control\.so"\s+name="gazebo_ros_control">\s*'
    r'.*?'
    r'</plugin>',
    re.DOTALL,
)

realsense_re = re.compile(
    r'<plugin\s+filename="librealsense_gazebo_plugin\.so"\s+name="realsense">\s*'
    r'.*?'
    r'</plugin>',
    re.DOTALL,
)


def laser_repl(match):
    old_name = match.group(1).strip() or "gazebo_ros_planar_lidar"
    topic = match.group(2).strip()
    frame = match.group(3).strip()
    plugin_name = f"{old_name}_{frame}".replace("/", "_")
    return f'''<plugin filename="libgazebo_ros_ray_sensor.so" name="{plugin_name}">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:={topic}</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>{frame}</frame_name>
      </plugin>'''


summary = []
for path in sorted(urdf_dir.glob("*.xacro")):
    text = path.read_text()
    original = text

    text, n_laser = gpu_laser_re.subn(laser_repl, text)
    text, n_control = control_re.subn(
        "<!-- ROS2 migration: disabled ROS1 libgazebo_ros_control.so; ros2_control migration pending. -->",
        text,
    )
    text, n_realsense = realsense_re.subn(
        "<!-- ROS2 migration: disabled ROS1 librealsense_gazebo_plugin.so; ROS2 camera/depth plugins will replace it later. -->",
        text,
    )

    if text != original:
        backup = path.with_name(path.name + f".bak_ros2_plugins_{stamp}")
        backup.write_text(original)
        path.write_text(text)
        summary.append((path, n_laser, n_control, n_realsense, backup))

if not summary:
    print("[INFO] No source xacro changed. It may already be patched.")
else:
    for path, n_laser, n_control, n_realsense, backup in summary:
        print(f"{path}: laser={n_laser}, ros_control={n_control}, realsense={n_realsense}")
        print(f"  backup: {backup}")
PY
echo

echo "[2/9] Build package so install/share uses patched xacros"
cd "${WS_DIR}"
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --packages-select "${PKG_NAME}" --symlink-install

echo "[3/9] Source workspace and prepare Gazebo environment"
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export GAZEBO_MASTER_URI="${MASTER_URI}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="${PKG_SRC}/models:${PKG_INSTALL}/models:${HOME}/.gazebo/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${PKG_INSTALL}:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

if [[ "${WORLD_KIND}" == "empty" ]]; then
  WORLD="${PKG_INSTALL}/worlds/empty.world"
else
  WORLD="${PKG_INSTALL}/worlds/blueberry_greenhouse.world"
fi
XACRO_FILE="${PKG_INSTALL}/urdf/husky_panda.urdf.xacro"
URDF_FILE="${OUT_DIR}/husky_panda_phase2.urdf"
GZ_LOG="${OUT_DIR}/gzserver.log"
SPAWN_LOG="${OUT_DIR}/spawn.log"

echo "WORLD=${WORLD}"
echo "XACRO_FILE=${XACRO_FILE}"
echo

echo "[4/9] Generate URDF from installed xacro and verify plugin state"
xacro "${XACRO_FILE}" > "${URDF_FILE}"
echo "--- plugin lines ---"
grep -n "<plugin\|libgazebo\|realsense_gazebo\|ros_control\|ray_sensor\|gpu_laser" "${URDF_FILE}" || true

if grep -qE "libgazebo_ros_control\.so|librealsense_gazebo_plugin\.so|libgazebo_ros_gpu_laser\.so" "${URDF_FILE}"; then
  echo "[ERROR] ROS1 plugins are still present in installed URDF. Build/install did not pick up the patch." >&2
  echo "Inspect:"
  echo "  grep -Rn \"libgazebo_ros_control\\|librealsense_gazebo_plugin\\|libgazebo_ros_gpu_laser\" ${PKG_SRC}/urdf ${PKG_INSTALL}/urdf"
  exit 2
fi

echo "[5/9] Clean old Gazebo/spawn processes and ROS2 daemon cache"
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f spawn_entity.py 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
sleep 3

echo "[6/9] Start gzserver headless"
rm -f "${GZ_LOG}" "${SPAWN_LOG}"
gzserver "${WORLD}" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  -s libgazebo_ros_force_system.so \
  --verbose > "${GZ_LOG}" 2>&1 &
GZ_PID=$!
echo "gzserver_pid=${GZ_PID}"
sleep 5

if ! ps -p "${GZ_PID}" >/dev/null 2>&1; then
  echo "[ERROR] gzserver exited early"
  tail -n 220 "${GZ_LOG}" || true
  exit 3
fi

echo "[7/9] Wait for typed /spawn_entity service"
python3 - <<'PY'
import time
import rclpy
from gazebo_msgs.srv import SpawnEntity

rclpy.init()
node = rclpy.create_node('phase2_spawn_ready_probe')
client = node.create_client(SpawnEntity, '/spawn_entity')

for i in range(60):
    if client.wait_for_service(timeout_sec=1.0):
        print(f'[OK] typed /spawn_entity ready after {i + 1}s')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    time.sleep(1.0)

print('[ERROR] typed /spawn_entity not ready within 60s')
for name, types in node.get_service_names_and_types():
    if 'spawn' in name or 'gazebo' in name:
        print(name, types)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1)
PY

echo "[8/9] Spawn original Husky + Panda from patched URDF"
set +e
timeout 150s ros2 run gazebo_ros spawn_entity.py \
  -entity husky \
  -file "${URDF_FILE}" \
  -timeout 120 \
  -x -1.5 \
  -y 0.0 \
  -z 0.35 \
  -Y 0.0 > "${SPAWN_LOG}" 2>&1
SPAWN_RC=$?
set -e

echo "--- spawn rc=${SPAWN_RC} ---"
cat "${SPAWN_LOG}" || true

echo "[9/9] Inspect topics/logs"
echo "--- ROS topics ---"
ros2 topic list | grep -E "clock|tf|joint_states|scan|laser|camera|image|depth|odom|robot_description" || true
echo "--- Gazebo topics ---"
gz topic -l 2>/dev/null | grep -E "husky|model|pose|laser|camera|realsense" || true
echo "--- gzserver log tail ---"
tail -n 260 "${GZ_LOG}" || true

if [[ "${SPAWN_RC}" -eq 0 ]]; then
  echo
  echo "[OK] Patched original Husky + Panda spawned in ${WORLD_KIND} world."
  echo "To inspect GUI:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
  echo "  gzclient --verbose"
else
  echo
  echo "[ERROR] Spawn failed or timed out. Paste this output."
fi

exit "${SPAWN_RC}"
