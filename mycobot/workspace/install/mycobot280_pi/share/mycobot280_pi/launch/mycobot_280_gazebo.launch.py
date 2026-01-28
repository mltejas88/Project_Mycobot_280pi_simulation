from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_ros_gz = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    pkg_mycobot = FindPackageShare("mycobot280_pi").find("mycobot280_pi")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_ros_gz, "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    urdf_path = PathJoinSubstitution([pkg_mycobot, "urdf", "mycobot_280_gazebo.urdf"])

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-file", urdf_path, "-name", "mycobot_280"],
        output="screen",
    )

    return LaunchDescription([gazebo, spawn])
