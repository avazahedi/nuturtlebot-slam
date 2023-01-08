from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # load model
        DeclareLaunchArgument(name='model', default_value=str((get_package_share_path('nuturtle_description') / 'urdf' / 'turtlebot3_burger.urdf.xacro')),
                              description='Absolute path to robot urdf file'),

        # launch rviz
        DeclareLaunchArgument(name='rvizconfig',
                              default_value=str((get_package_share_path('nuturtle_description') / 'config' / 'basic_purple.rviz')),
                              description='Path to rviz config file'),

        # rviz launch argument
        DeclareLaunchArgument(name='use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Launch rviz if use_rviz is true'),

        # joint state publisher launch argument
        DeclareLaunchArgument(name='use_jsp', default_value='true',
                              choices=['true', 'false'],
                              description='Launch joint_state_publisher_gui '
                                        'if view_only is true'),

        ## Nodes ##
        # robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                Command([ExecutableInPackage("xacro", "xacro"), " ",
                        PathJoinSubstitution(
                        [FindPackageShare("nuturtle_description"), "urdf", "turtlebot3_burger.urdf.xacro"])])}
            ]
        ),

        # launch rviz if use_rviz is true
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        ),

        # joint state publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=LaunchConfigurationEquals('use_jsp', 'true')
        )

    ])
