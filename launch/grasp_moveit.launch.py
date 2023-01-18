from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node







def generate_launch_description():


    grasp_moveit = Node(
            package='grasp_moveit',
            executable='grasp_moveit',
            emulate_tty=True,
            parameters=[{}],
            output='screen')

    return LaunchDescription([
        grasp_moveit
    ])
