from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = "turtlebot3_perception"

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation/Gazebo clock")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    lmark_slam = Node(
        name="landmarks_slam",
        package=package_name,
        executable="landmarks_slam",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([declare_use_sim_time, slam, lmark_slam])
