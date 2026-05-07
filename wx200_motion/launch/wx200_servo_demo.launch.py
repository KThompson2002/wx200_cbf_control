import os

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from launch_param_builder import ParameterBuilder

def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    with open(os.path.join(pkg_path, file_path), 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    hardware_type = LaunchConfiguration('hardware_type')
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    robot_description_arg = LaunchConfiguration('robot_description')
    mode_configs_arg = LaunchConfiguration('mode_configs')

    use_sim_time = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type,
    )

    robot_description = {'robot_description': robot_description_arg}

    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'), 'config',
    ])
    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model='wx200',
                config_path=config_path,
            ),
    }

    # ── MoveIt configs (all from interbotix_xsarm_moveit) ──────────────────
    kinematics_config = PathJoinSubstitution([
        FindPackageShare('wx200_motion'), 'config', 'kinematics.yaml',
    ])

    ompl_yaml = load_yaml('interbotix_xsarm_moveit', 'config/ompl_planning.yaml')
    ompl_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                'default_planning_request_adapters/CheckForStackedConstraints',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
            ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_config['move_group'].update(ompl_yaml)

    controllers_config = load_yaml(
        'interbotix_xsarm_moveit', 'config/controllers/wx200_controllers.yaml'
    )
    joint_limits = {
        'robot_description_planning': load_yaml(
            'interbotix_xsarm_moveit', 'config/joint_limits/wx200_joint_limits.yaml'
        )
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_ros_control'),
                'launch', 'xsarm_ros_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': 'wx200',
            'robot_name': robot_name,
            'use_rviz': 'false',
            'mode_configs': mode_configs_arg,
            'hardware_type': hardware_type,
            'robot_description': robot_description_arg,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    move_group_remappings = [
        ('joint_states', f'/{robot_name}/joint_states'),
        (f'{robot_name}/get_planning_scene', f'/{robot_name}/get_planning_scene'),
        ('/arm_controller/follow_joint_trajectory',
         f'/{robot_name}/arm_controller/follow_joint_trajectory'),
        ('/gripper_controller/follow_joint_trajectory',
         f'/{robot_name}/gripper_controller/follow_joint_trajectory'),
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': f'/{robot_name}/joint_states',
                },
                'use_sim_time': use_sim_time,
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            joint_limits,
            {'sensors': ['']},
        ],
        remappings=move_group_remappings,
        output='screen',
    )

    # ── RViz (optional) ─────────────────────────────────────────────────────
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_config,
            kinematics_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=move_group_remappings,
        output='log',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare(
                    'wx200_motion'
                ), 'config', 'motion.rviz']
            ),
        ],
    )

    position_server_node = Node(
        package='wx200_motion',
        executable='position_server',
        name='position_server',
        parameters=[{
            'planning_group': 'interbotix_arm',
            'ee_link': f'{robot_name}/ee_gripper_link',
            'default_velocity_scaling': 0.5,
            'default_acceleration_scaling': 0.5,
            'planning_time': 5.0,
            'move_group_action': '/move_action',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/panda_simulated_config.yaml",
        )
        .to_dict()
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    servo_node = Node(
        package="realtime_servo",
        executable="servo_cpp_interface_demo",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
    )

    return [
        # sanitizer_node,
        move_group_node,
        rviz_node,
        position_server_node,
        TimerAction(period=8.0, actions=[hardware_launch]),
        TimerAction(period=14.0, actions=[servo_node]),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='wx200',
            description='Interbotix arm model type.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='wx200',
            description='Name of the robot (typically equal to robot_model).',
        ),
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit_interface'),
                'config', 'modes.yaml',
            ]),
            description="Path to the xs_sdk mode config YAML.",
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='Launch RViz with MoveIt visualization.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description='Use simulation clock.',
        ),
    ]
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


