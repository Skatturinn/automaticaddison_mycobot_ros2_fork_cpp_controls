from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Tell the builder the name of the robot (for the .srdf file) AND the exact package name
    moveit_config = MoveItConfigsBuilder(
        "mycobot_280", 
        package_name="mycobot_moveit_config_ik"
    ).to_moveit_configs()

    ik_streamer_node = Node(
        package='mycobot_system_tests', 
        executable='ik_streamer_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([ik_streamer_node])
