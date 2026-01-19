import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_pkg_share = get_package_share_directory('kine_robot_description')

    urdf_path = os.path.join(robot_pkg_share, 'urdf', 'crane3r.urdf')
    srdf_path = os.path.join(robot_pkg_share, 'config', 'crane3r.srdf')
    kinematics_path = os.path.join(robot_pkg_share, 'config', 'kinematics.yaml')
    joint_limits_path = os.path.join(robot_pkg_share, 'config', 'joint_limits.yaml')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    with open(srdf_path, 'r') as f:
        robot_description_semantic_content = f.read()

    with open(kinematics_path, 'r') as f:
        kinematics_content = f.read()

    with open(joint_limits_path, 'r') as f:
        joint_limits_content = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    moveit_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content},
            {'robot_description_kinematics': kinematics_content},
            {'robot_description_planning': {'joint_limits': joint_limits_content}}
        ]
    )

    kine_env_node = Node(
        package='kine',
        executable='kine_environment',
        name='kine_environment',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': robot_description_content}]
    )

    return LaunchDescription([
        rsp_node, moveit_node, kine_env_node
    ])
