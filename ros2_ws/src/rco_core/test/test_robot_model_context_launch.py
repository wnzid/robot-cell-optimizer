import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import xacro
import yaml


def load_yaml(package_name: str, relative_path: str):
    package_share = get_package_share_directory(package_name)
    path = os.path.join(package_share, relative_path)

    with open(path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)


@pytest.mark.launch_test
def generate_test_description():
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

    test_node = Node(
        package='rco_core',
        executable='test_robot_model_context',
        name='rco_robot_model_context_test',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    keep_alive = ExecuteProcess(
        cmd=['sleep', '3'],
        output='screen',
    )

    return (
        launch.LaunchDescription(
            [
                test_node,
                keep_alive,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'test_node': test_node},
    )


class TestRobotModelContextProcess(unittest.TestCase):

    def test_process_finishes(self, proc_info, test_node):
        proc_info.assertWaitForShutdown(
            process=test_node,
            timeout=45,
        )


@launch_testing.post_shutdown_test()
class TestRobotModelContextExitCode(unittest.TestCase):

    def test_exit_code(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=test_node,
        )
