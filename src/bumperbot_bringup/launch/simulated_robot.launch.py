import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # Gazebo: Start immediately
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
    )

    # Controller: Delay 10s
    controller = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bumperbot_controller"),
                        "launch",
                        "controller.launch.py"
                    )
                ),
                launch_arguments={
                    "use_simple_controller": "False",
                    "use_python": "False"
                }.items()
            )
        ]
    )

    # Joystick: Delay 6s
    joystick = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bumperbot_controller"),
                        "launch",
                        "joystick_teleop.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": "True"
                }.items()
            )
        ]
    )

    # Localization or SLAM: Delay 8s
    localization = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bumperbot_localization"),
                        "launch",
                        "global_localization.launch.py"
                    )
                ),
                condition=UnlessCondition(use_slam)
            )
        ]
    )

    slam = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bumperbot_mapping"),
                        "launch",
                        "slam.launch.py"
                    )
                ),
                condition=IfCondition(use_slam)
            )
        ]
    )

    # Navigation: Delay 10s
    navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bumperbot_navigation"),
                        "launch",
                        "navigation.launch.py"
                    )
                )
            )
        ]
    )

    # RViz: Delay 12s
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "rviz",
                    "nav2_default_view.rviz"
                )],
                output="screen",
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        rviz,
    ])
