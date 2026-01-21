import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory('kine_robot_description'), 'urdf', 'crane3r.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_content}]
    )

    kine_control_node = Node(
        package='kine',
        executable='kine_control',
        name='kine_control',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_content}]
    )

    vision_node = Node(
        package='kine_vision',
        executable='kine_vision',
        name='kine_vision',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rsp_node, kine_env_node, kine_control_node, vision_node
    ])
