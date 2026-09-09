import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(package_name: str, relative_path: str):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, relative_path)

    with open(absolute_path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    ur_description_share = get_package_share_directory('ur_description')
    ur_moveit_share = get_package_share_directory('ur_moveit_config')

    urdf_path = os.path.join(
        ur_description_share,
        'urdf',
        'ur.urdf.xacro',
    )

    srdf_path = os.path.join(
        ur_moveit_share,
        'srdf',
        'ur.srdf.xacro',
    )

    robot_description = {
        'robot_description': xacro.process_file(
            urdf_path,
            mappings={
                'ur_type': 'ur5e',
                'name': 'ur5e',
            },
        ).toxml()
    }

    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            srdf_path,
            mappings={
                'name': 'ur5e',
            },
        ).toxml()
    }

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(
            'ur_moveit_config',
            'config/kinematics.yaml',
        )
    }

    diagnostics = Node(
        package='rco_core',
        executable='robot_diagnostics',
        name='rco_robot_diagnostics',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([diagnostics])
