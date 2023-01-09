from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, \
                                    TextSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # color argument
        DeclareLaunchArgument(name='color', default_value='purple',
                              choices=['red', 'green', 'blue', 'purple'],
                              description='Robot base_link color'),

        # rviz color option
        SetLaunchConfiguration(name='rviz_color',
                               value=[TextSubstitution(text='basic_'),
                                      LaunchConfiguration('color'),
                                      TextSubstitution(text='.rviz')]),

        # launch rviz
        DeclareLaunchArgument(name='rvizconfig',
                              default_value=PathJoinSubstitution(
                                [FindPackageShare("nuturtle_description"), "config",
                                 LaunchConfiguration('rviz_color')]),
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

        # frame prefix arg
        SetLaunchConfiguration(name='frame_prefix_arg',
                               value=[LaunchConfiguration('color'),
                                      TextSubstitution(text='/')]),

        # fixed frame arg
        SetLaunchConfiguration(name='fixedf_arg',
                               value=[LaunchConfiguration('color'),
                                      TextSubstitution(text='/base_link')]),

        # Nodes #
        # robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration('color'),
            parameters=[
                {"robot_description":
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                         PathJoinSubstitution(
                         [FindPackageShare("nuturtle_description"), "urdf",
                          "turtlebot3_burger.urdf.xacro"]),
                         " color:=",
                         LaunchConfiguration('color')
                        ]),
                 "frame_prefix": LaunchConfiguration('frame_prefix_arg')}],
        ),

        # launch rviz if use_rviz is true
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=LaunchConfiguration('color'),
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig'), '-f',
                       LaunchConfiguration('fixedf_arg')],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        ),

        # joint state publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=LaunchConfiguration('color'),
            condition=LaunchConfigurationEquals('use_jsp', 'true')
        )

    ])
