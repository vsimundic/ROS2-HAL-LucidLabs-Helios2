from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    debug = LaunchConfiguration('debug')
    params = [os.path.join(
        get_package_share_directory("helios2_hal"),
        "params",
        "helios2_ray_params.yaml"
    )]

    node_debug = Node(
        package='helios2_hal',
        executable='HALHeliosExecutable',
        name='helios2_hal',
        output='screen',
        emulate_tty=True,
        parameters=params,
        prefix=['gdbserver --once localhost:3000'],
        condition=IfCondition(debug),
    )

    node_normal = Node(
        package='helios2_hal',
        executable='HALHeliosExecutable',
        name='helios2_hal',
        output='screen',
        emulate_tty=True,
        parameters=params,
        condition=UnlessCondition(debug),
    )

    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        node_debug,
        node_normal,
    ])