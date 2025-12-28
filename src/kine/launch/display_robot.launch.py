import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'kine'
    pkg_share = get_package_share_directory(pkg)

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_envvironment',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    kine_controller_node = Node(
        package='kine',
        executable='kine_control',
        name='kine_control',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    interactive_goal_node = Node(
        package='interactive_goal',
        executable='interactive_goal_node',
        name='interactive_goal_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rsp_node,
        rviz_node,
        kine_env_node,
        kine_controller_node,
        interactive_goal_node
    ])
