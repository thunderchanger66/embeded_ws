from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('armmodel'),
        'urdf',
        'armmodel.urdf'
    )

    rviz2_config = os.path.join(
        get_package_share_directory('armmodel'),
        'rviz',
        'armmodel.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value='',
            description='Robot model file'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d',rviz2_config],
            output='screen'
        )
    ])