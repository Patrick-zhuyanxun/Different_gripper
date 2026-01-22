from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Package paths
    moveit_config_pkg = 'three_finger_soft_moveit_config'
    gripper_description_pkg = 'gripper_description'
    
    # Get package share directories
    moveit_config_share = get_package_share_directory(moveit_config_pkg)
    gripper_description_share = get_package_share_directory(gripper_description_pkg)
    
    # Generate robot description from xacro
    robot_description_content = Command([
        'xacro ',
        os.path.join(gripper_description_share, 'urdf', 'tm5_with_three_finger_gripper.xacro')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # SRDF
    robot_description_semantic_path = os.path.join(
        moveit_config_share, 'config', 'tm5_with_gripper.srdf'
    )
    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    
    # Load kinematics
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    
    # Load joint limits
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    joint_limits = {'robot_description_planning': joint_limits_yaml}
    
    # Load OMPL planning
    ompl_planning_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                              'default_planner_request_adapters/ResolveConstraintFrames '
                              'default_planner_request_adapters/FixWorkspaceBounds '
                              'default_planner_request_adapters/FixStartStateBounds '
                              'default_planner_request_adapters/FixStartStateCollision '
                              'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    
    # Trajectory execution config
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Planning scene monitor config
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # RViz config
    rviz_config_file = os.path.join(moveit_config_share, 'config', 'moveit.rviz')
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # Move Group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning_pipeline_config,
        ],
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        move_group_node,
        rviz_node,
    ])
