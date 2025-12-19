import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class WorldType:
    RMUC = "RMUC"
    RMUL = "RMUL"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation clock"
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=WorldType.RMUL,
        description="Choose <RMUC> or <RMUL>",
    )

    pb_rm_sim_launch_dir = os.path.join(
        get_package_share_directory("pb_rm_simulation"), "launch"
    )
    rm_nav_bringup_dir = get_package_share_directory("rm_nav_bringup")

    # 1) Gazebo world + robot spawn (provides /livox/lidar/pointcloud + /livox/imu + TF)
    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pb_rm_sim_launch_dir, "rm_simulation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "rviz": "False",
        }.items(),
    )

    # 2) Build /scan from Livox pointcloud (same nodes/params as rm_nav_bringup/bringup_sim.launch.py)
    segmentation_params = os.path.join(
        rm_nav_bringup_dir, "config", "simulation", "segmentation_sim.yaml"
    )

    ground_segmentation = Node(
        package="linefit_ground_segmentation_ros",
        executable="ground_segmentation_node",
        output="screen",
        parameters=[segmentation_params],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", ["/segmentation/obstacle"]), ("scan", ["/scan"])],
        parameters=[
            {
                "target_frame": "livox_frame",
                "transform_tolerance": 0.01,
                "min_height": -1.0,
                "max_height": 0.1,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0043,
                "scan_time": 0.3333,
                "range_min": 0.45,
                "range_max": 10.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        name="pointcloud_to_laserscan",
        output="screen",
    )

    # 3) Your controller (10Hz). Publishes to /cmd_vel_chassis (Gazebo planar_move consumes it).
    rl_controller = Node(
        package="rm_rl_control",
        executable="rl_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "control_rate_hz": 10.0,
                "scan_topic": "/scan",
                "imu_topic": "/livox/imu",
                "goal_topic": "/rl_goal",
                "cmd_vel_topic": "/cmd_vel_chassis",
                "base_frame": "base_link",
            }
        ],
    )

    # Optional: allow selecting dynamic obstacles world via manual override (kept in pb_rm_simulation launch)
    rmuc_group = GroupAction(
        condition=LaunchConfigurationEquals("world", WorldType.RMUC),
        actions=[],
    )
    rmul_group = GroupAction(
        condition=LaunchConfigurationEquals("world", WorldType.RMUL),
        actions=[],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_world,
            start_sim,
            ground_segmentation,
            pointcloud_to_laserscan,
            rl_controller,
            rmuc_group,
            rmul_group,
        ]
    )

