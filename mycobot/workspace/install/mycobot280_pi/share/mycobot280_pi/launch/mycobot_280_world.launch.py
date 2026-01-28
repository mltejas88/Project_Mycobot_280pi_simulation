from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Publish the URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(
                    '/home/tejas/mycobot/workspace/src/mycobot280_pi/urdf/mycobot_280_gazebo.urdf'
                ).read()
            }]
        ),

        # Launch Gazebo and spawn the robot
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'mycobot280_pi', 'mycobot_280_gazebo.launch.py'
            ],
            output='screen'
        ),

        # Spawn controllers after Gazebo + robot are up
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'mycobot280_pi', 'gz_clock_bridge.launch.py'
            ],
            output='screen'
        ),
    ])
