#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="${1:-/home/wjkxy/ros2_ws/src/fkie_husky_manipulation_simulation}"
LAUNCH_FILE="${PKG_DIR}/launch/demo_blueberry_ros2_file_spawn.launch.py"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -f "${LAUNCH_FILE}" ]]; then
  echo "[ERROR] Launch file not found: ${LAUNCH_FILE}" >&2
  echo "Usage: $0 /path/to/fkie_husky_manipulation_simulation" >&2
  exit 1
fi

cp "${LAUNCH_FILE}" "${LAUNCH_FILE}.bak_spawn_service_${STAMP}"

python3 - "${LAUNCH_FILE}" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()

if "gazebo_plugin_path = ':'.join(plugin_paths)" not in text:
    old = """    old_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    resource_paths = [
        pkg_share,
        '/usr/share/gazebo-11',
    ]
    if old_resource_path:
        resource_paths.append(old_resource_path)
    gazebo_resource_path = ':'.join(resource_paths)

    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
"""
    new = """    old_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    resource_paths = [
        pkg_share,
        '/usr/share/gazebo-11',
    ]
    if old_resource_path:
        resource_paths.append(old_resource_path)
    gazebo_resource_path = ':'.join(resource_paths)

    old_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    plugin_paths = [
        '/opt/ros/humble/lib',
        '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
    ]
    if old_plugin_path:
        plugin_paths.append(old_plugin_path)
    gazebo_plugin_path = ':'.join(plugin_paths)

    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
"""
    if old not in text:
        raise SystemExit('[ERROR] Could not find resource-path block to patch')
    text = text.replace(old, new, 1)

if "name='GAZEBO_PLUGIN_PATH'" not in text:
    old = """        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=gazebo_resource_path,
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value='',
        ),
"""
    new = """        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=gazebo_resource_path,
        ),

        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=gazebo_plugin_path,
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value='',
        ),
"""
    if old not in text:
        raise SystemExit('[ERROR] Could not find environment block to patch')
    text = text.replace(old, new, 1)

text = text.replace("'verbose': 'false',", "'verbose': 'true',")
text = text.replace("period=6.0,", "period=12.0,")

if "'-timeout'," not in text:
    old = """                        '-entity', 'husky',
                        '-file', urdf_file,
                        '-x', LaunchConfiguration('x'),
"""
    new = """                        '-entity', 'husky',
                        '-file', urdf_file,
                        '-timeout', '120',
                        '-x', LaunchConfiguration('x'),
"""
    if old not in text:
        raise SystemExit('[ERROR] Could not find spawn_entity argument block to patch')
    text = text.replace(old, new, 1)

path.write_text(text)
PY

chmod +x "${LAUNCH_FILE}"

echo "[OK] Spawn service stability patch applied."
echo "[OK] Backup:"
echo "  ${LAUNCH_FILE}.bak_spawn_service_${STAMP}"
echo
echo "Before launching, clean stale Gazebo processes in the ROS2 terminal:"
echo "  pkill -f gzserver || true"
echo "  pkill -f gzclient || true"
echo
echo "Then run:"
echo "  cd /home/wjkxy/ros2_ws"
echo "  colcon build --packages-select fkie_husky_manipulation_simulation --symlink-install"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  export GAZEBO_MASTER_URI=http://127.0.0.1:11345"
echo "  ros2 launch fkie_husky_manipulation_simulation demo_blueberry_ros2_file_spawn.launch.py gui:=true"
echo
echo "If spawn still waits, run in another terminal:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /home/wjkxy/ros2_ws/install/setup.bash"
echo "  ros2 service list | grep spawn"
echo "  ps -ef | grep -E 'gzserver|gzclient' | grep -v grep"
echo "  tail -n 120 ~/.ros/log/latest/gzserver-*.log"
