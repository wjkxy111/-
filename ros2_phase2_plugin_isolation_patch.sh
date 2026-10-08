#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="${1:-/home/wjkxy/ros2_ws/src/fkie_husky_manipulation_simulation}"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -d "${PKG_DIR}/urdf" ]]; then
  echo "[ERROR] URDF directory not found: ${PKG_DIR}/urdf" >&2
  echo "Usage: $0 /path/to/fkie_husky_manipulation_simulation" >&2
  exit 1
fi

python3 - "${PKG_DIR}" "${STAMP}" <<'PY'
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


changed = []
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
        changed.append(str(path))
        summary.append((path.name, n_laser, n_control, n_realsense, str(backup)))

print("=== changed files ===")
for item in changed:
    print(item)

print("=== replacement summary ===")
for name, n_laser, n_control, n_realsense, backup in summary:
    print(f"{name}: laser={n_laser}, ros_control={n_control}, realsense={n_realsense}")
    print(f"  backup: {backup}")

if not changed:
    print("[WARN] No xacro files changed. The plugin text may differ from expected patterns.")
PY

echo
echo "[OK] ROS1 Gazebo plugin isolation patch finished."
echo
echo "Now run these commands in the ROS2 terminal:"
echo "  cd /home/wjkxy/ros2_ws"
echo "  colcon build --packages-select fkie_husky_manipulation_simulation --symlink-install"
echo "  bash /mnt/c/Users/34542/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/ros2_phase1_spawn_debug.sh"
echo
echo "Expected in generated URDF after rebuild:"
echo "  no libgazebo_ros_control.so"
echo "  no librealsense_gazebo_plugin.so"
echo "  libgazebo_ros_ray_sensor.so for laser_front and laser_rear"
