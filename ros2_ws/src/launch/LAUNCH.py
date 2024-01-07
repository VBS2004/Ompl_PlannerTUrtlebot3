from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_motion_planning',
            executable='/root/Ompl_PlannerTurtlebot3/ros2_ws/build/my_robot_motion_planning/my_motion_planner',
            name='my_motion_planner',
            output='screen',
            parameters=[{'map_file': '/root/Ompl_PlannerTurtlebot3/ros2_ws/src/maps/your_map.yaml'}]  # Set the default value here
        )
    ])

