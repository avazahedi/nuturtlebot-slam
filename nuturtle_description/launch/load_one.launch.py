"""Launch differential drive robot in gazebo with rviz."""
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    """Generate launch description for launching robot in gazebo with rviz."""
    nuturtle_desc_path = get_package_share_path('nuturtle_description')
    default_model_path = nuturtle_desc_path / 'turtlebot3_burger.urdf.xacro'
    default_rviz_config_path = nuturtle_desc_path / 'basic_purple.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')

    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(default_rviz_config_path),
                                     description='Path to rviz config file')

    use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='true',
                                    choices=['true', 'false'],
                                    description='Launch rviz if use_rviz is true')

    jsp_arg = DeclareLaunchArgument(name='use_jsp', default_value='true',
                                    choices=['true', 'false'],
                                    description='Launch joint_state_publisher_gui '
                                                'if view_only is true')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Launch rviz if use_rviz is true
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=LaunchConfigurationEquals('use_rviz', 'true'),
        on_exit=Shutdown()
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('use_jsp', 'true')
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_rviz,
        jsp_arg,
        robot_state_publisher,
        rviz_launch,
        joint_state_publisher_gui_node
    ])
