from launch import LaunchDescription
from launch_ros.actions import Node 
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        robot_name="robot_gripper", 
        package_name="moveit_robot_setup").to_moveit_configs()
    
    return LaunchDescription([
        Node(
            package="robot_move",
            executable="robot_move_execute",
            output="screen",
            parameters=[moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        {"use_sim_time": True}],
        )
    ])
