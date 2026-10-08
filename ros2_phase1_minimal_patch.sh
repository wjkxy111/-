#!/usr/bin/env bash
set -euo pipefail

PKG_DIR="${1:-/home/wjkxy/ros2_ws/src/fkie_husky_manipulation_simulation}"
LAUNCH_FILE="${PKG_DIR}/launch/demo_blueberry_ros2_file_spawn.launch.py"
PACKAGE_XML="${PKG_DIR}/package.xml"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -d "${PKG_DIR}" ]]; then
  echo "[ERROR] Package directory not found: ${PKG_DIR}" >&2
  echo "Usage: $0 /path/to/fkie_husky_manipulation_simulation" >&2
  exit 1
fi

if [[ ! -f "${LAUNCH_FILE}" ]]; then
  echo "[ERROR] Launch file not found: ${LAUNCH_FILE}" >&2
  exit 1
fi

if [[ ! -f "${PACKAGE_XML}" ]]; then
  echo "[ERROR] package.xml not found: ${PACKAGE_XML}" >&2
  exit 1
fi

cp "${LAUNCH_FILE}" "${LAUNCH_FILE}.bak_phase1_${STAMP}"
cp "${PACKAGE_XML}" "${PACKAGE_XML}.bak_phase1_${STAMP}"

cat > "${LAUNCH_FILE}" <<'EOF'
#!/usr/bin/env python3
import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def prepare_urdf(context, *args, **kwargs):
    pkg_name = 'fkie_husky_manipulation_simulation'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, 'urdf', 'husky_panda.urdf.xacro')
    urdf_file = '/tmp/husky_panda_ros2.urdf'

    cmd = ['xacro', xacro_file]

    print('[demo_blueberry_ros2_file_spawn] generating URDF:')
    print('  xacro:', xacro_file)
    print('  urdf :', urdf_file)

    with open(urdf_file, 'w') as f:
        subprocess.check_call(cmd, stdout=f)

    print('[demo_blueberry_ros2_file_spawn] URDF generated OK.')
    return []


def generate_launch_description():
    pkg_name = 'fkie_husky_manipulation_simulation'

    pkg_share = get_package_share_directory(pkg_name)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'husky_panda.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'blueberry_greenhouse.world')
    urdf_file = '/tmp/husky_panda_ros2.urdf'

    pkg_models = os.path.join(pkg_share, 'models')
    src_models = os.path.expanduser('~/ros2_ws/src/fkie_husky_manipulation_simulation/models')
    user_models = os.path.expanduser('~/.gazebo/models')
    gazebo_default_models = '/usr/share/gazebo-11/models'

    old_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_paths = [
        src_models,
        pkg_models,
        user_models,
        gazebo_default_models,
    ]
    if old_model_path:
        model_paths.append(old_model_path)
    gazebo_model_path = ':'.join(model_paths)

    old_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    resource_paths = [
        pkg_share,
        '/usr/share/gazebo-11',
    ]
    if old_resource_path:
        resource_paths.append(old_resource_path)
    gazebo_resource_path = ':'.join(resource_paths)

    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='-1.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.35'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'gazebo_master_uri',
            default_value='http://127.0.0.1:11345',
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MASTER_URI',
            value=LaunchConfiguration('gazebo_master_uri'),
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path,
        ),

        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=gazebo_resource_path,
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value='',
        ),

        OpaqueFunction(function=prepare_urdf),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': robot_description,
                }
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_file,
                'gui': LaunchConfiguration('gui'),
                'verbose': 'false',
            }.items(),
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_husky_panda_from_file',
                    output='screen',
                    arguments=[
                        '-entity', 'husky',
                        '-file', urdf_file,
                        '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'),
                        '-Y', LaunchConfiguration('yaw'),
                    ],
                )
            ],
        ),
    ])
EOF

python3 - "${PACKAGE_XML}" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()

deps = [
    'husky_description',
    'franka_description',
    'fkie_realsense_description',
]

insert_marker = '  <export>\n'
if insert_marker not in text:
    raise SystemExit('[ERROR] package.xml does not contain the expected <export> marker')

for dep in deps:
    line = f'  <exec_depend>{dep}</exec_depend>\n'
    if f'<exec_depend>{dep}</exec_depend>' not in text:
        text = text.replace(insert_marker, line + insert_marker, 1)

path.write_text(text)
PY

chmod +x "${LAUNCH_FILE}"

echo "[OK] Phase 1 minimal patch applied."
echo "[OK] Backups:"
echo "  ${LAUNCH_FILE}.bak_phase1_${STAMP}"
echo "  ${PACKAGE_XML}.bak_phase1_${STAMP}"
echo
echo "Next commands:"
echo "  cd /home/wjkxy/ros2_ws"
echo "  colcon build --packages-select fkie_husky_manipulation_simulation --symlink-install"
echo "  source install/setup.bash"
echo "  export GAZEBO_MASTER_URI=http://127.0.0.1:11345"
echo "  ros2 launch fkie_husky_manipulation_simulation demo_blueberry_ros2_file_spawn.launch.py gui:=true"
echo
echo "Checks in another terminal:"
echo "  source /home/wjkxy/ros2_ws/install/setup.bash"
echo "  ros2 topic list | grep -E 'robot_description|tf|joint_states|scan|laser|camera|image|depth|odom'"
echo "  ros2 param get /robot_state_publisher robot_description"
echo "  gz topic -l"
