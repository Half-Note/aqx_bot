import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    world_name = LaunchConfiguration("world_name")

    
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bumperbot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        )
    )

    imu_driver_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="aqxbot_firmware",
                executable="imu_receiver",
                name="imu_receiver",
                output="screen"
            )
        ]
    )

    rplidar_receiver_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="aqxbot_firmware",
                executable="rplidar_receiver",
                name="rplidar_receiver",
                output="screen"
            )
        ]
    )

    controller = TimerAction(
        period=3.0,
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

    joystick = TimerAction(
        period=4.0,
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
                    "use_sim_time": "False"
                }.items()
            )
        ]
    )

    slam = TimerAction(
        period=2.0,
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

    localization = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bumperbot_localization"),
                        "launch",
                        "global_localization.launch.py"
                    )
                ),
                launch_arguments=[
                    ("map_name", world_name)  # <-- pass world_name as map_name
                ],
                condition=UnlessCondition(use_slam)
            )
        ]
    )



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
                parameters=[{"use_sim_time": False}]
            )
        ]
    )

    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        imu_driver_node,
        rplidar_receiver_node,
        controller,
        joystick,
        slam,
        localization,
        navigation,
        rviz
    ])
