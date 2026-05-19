from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    Command
)
# from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
import os, subprocess
import yaml
from launch_param_builder import ParameterBuilder
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def load_yaml(package_name, file_path):
    """Load a YAML file from a ROS package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_args():
    jacobian_base_link_arg = DeclareLaunchArgument(
        name='jacobian_base_link',
        default_value='wx200/base_link',
        description='Base link for Jacobian chain'
    )
    jacobian_ee_link_arg = DeclareLaunchArgument(
        name='jacobian_ee_link',
        default_value='wx200/ee_gripper_link',
        description='End-effector link for Jacobian chain'
    )
    jacobian_joint_states_topic_arg = DeclareLaunchArgument(
        name='jacobian_joint_states_topic',
        default_value='/wx200/joint_states',
        description='Joint states topic consumed by get_jacobian_node'
    )
    control_rate_arg = DeclareLaunchArgument(
        name='control_rate_hz',
        default_value='30.0',
        description='Frequency for velocity ctrl (Hz)'
    )
    control_mode_arg = DeclareLaunchArgument(
        name='control_mode',
        default_value='position',
        description='Jacobian output mode: position or velocity'
    )
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
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description='Optional path to an additional SRDF xacro to include.',
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
    launch_args = [
        jacobian_base_link_arg,
        jacobian_ee_link_arg,
        jacobian_joint_states_topic_arg,
        control_rate_arg,
        control_mode_arg,
    ]
    return launch_args + declared_arguments

# def load_yaml(package_name, file_path):
#     pkg_path = get_package_share_directory(package_name)
#     with open(os.path.join(pkg_path, file_path), 'r') as f:
#         return yaml.safe_load(f)

def launch_setup(context):
    # ==========================================================================
    # Load Configurations
    # ==========================================================================
    # piper_moveit_config_path = get_package_share_directory('piper_moveit_config')
    # # if use_gripper is false, load the no_gripper version of the xacro and configs
    # use_gripper = LaunchConfiguration('use_gripper').perform(context)
    # if use_gripper.lower() == 'false':
    #     urdf_file = os.path.join(piper_moveit_config_path, 'config/no_gripper', 'piper.urdf.xacro')
    #     srdf_file = load_file('piper_moveit_config', 'config/no_gripper/piper.srdf')
    #     initial_positions_file = os.path.join(piper_moveit_config_path, "config/no_gripper", "initial_positions.yaml")
    #     # Controller configuration
    #     controllers_yaml_path = os.path.join(piper_moveit_config_path, "config/no_gripper", "ros2_controllers.yaml")
    #     # Kinematics configuration
    #     kinematics_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/kinematics.yaml')
    #     joint_limits_yaml = load_yaml('piper_moveit_config', 'config/no_gripper/joint_limits.yaml')
    #     joint_limits_yaml_path = os.path.join(piper_moveit_config_path, "config/no_gripper", "joint_limits.yaml")
    # else:
    #     urdf_file = os.path.join(piper_moveit_config_path, 'config', 'piper.urdf.xacro')
    #     srdf_file = load_file('piper_moveit_config', 'config/piper.srdf')
    #     initial_positions_file = os.path.join(piper_moveit_config_path, "config", "initial_positions.yaml")
    #     # Controller configuration
    #     controllers_yaml_path = os.path.join(piper_moveit_config_path, "config", "ros2_controllers.yaml")
    #     # Kinematics configuration
    #     kinematics_yaml = load_yaml('piper_moveit_config', 'config/kinematics.yaml')
    #     joint_limits_yaml = load_yaml('piper_moveit_config', 'config/joint_limits.yaml')
    #     joint_limits_yaml_path = os.path.join(piper_moveit_config_path, "config", "joint_limits.yaml")
    
    # robot_description_config = xacro.process_file(
    #     urdf_file,
    #     mappings={"initial_positions_file": initial_positions_file})
    # robot_description = {"robot_description": robot_description_config.toxml()}
    # # Semantic description (SRDF)
    # robot_description_semantic = {'robot_description_semantic': srdf_file}

    joint_limits = {
        'robot_description_planning': load_yaml(
            'interbotix_xsarm_moveit', 'config/joint_limits/wx200_joint_limits.yaml'
        )
    }

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
    kinematics_yaml = load_yaml('wx200_motion', 'config/kinematics.yaml')
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}


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

    # moveit_controllers = {
    #     'moveit_simple_controller_manager': controllers_config,
    #     'moveit_controller_manager':
    #         'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    # }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 3.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.2,
        'trajectory_execution.allowed_start_tolerance': 0.1,
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

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('wx200_motion'), 'config', 'kinematics.yaml',
    ])

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

    # xs_control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([FindPackageShare('interbotix_xsarm_control'),
    #                               'launch', 'xsarm_control.launch.py'])
    #     ]),
    #     launch_arguments={
    #         'robot_model': 'wx200',
    #         'robot_name': robot_name,
    #         'mode_configs':mode_configs_arg,
    #         'hardware_type': hardware_type,
    #         'robot_description': robot_description_arg,
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )

    # ros2_control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     namespace=robot_name,
    #     parameters=[
    #         {'robot_description': robot_description_arg},
    #         PathJoinSubstitution([FindPackageShare('interbotix_xsarm_ros_control'),
    #                               'config', 'controllers', 'wx200_controllers.yaml']),
    #         PathJoinSubstitution([FindPackageShare('realtime_servo'),
    #                               'config', 'controller_manager_override.yaml']),
    #     ],
    #     output={'both': 'screen'},
    # )

    # arm_controller_spawner = Node(
    #     name='arm_controller_spawner',
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace=robot_name,
    #     arguments=['-c', f'/{robot_name}/controller_manager', 'arm_controller'],
    #     output={'both': 'screen'},
    # ) 


    rviz_config_file = os.path.join(get_package_share_directory('realtime_servo'), 'config', 'robot_only.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # planning_scene_monitor
        ]
    )

    cbf_filter_node = Node(
        package='wx200_motion',
        executable='cbf_filter',
        name='cbf_filter',
        output='screen',
        parameters=[{
            'input_topic': '/velocity_pub/vel_command',
            'output_topic': '/wx200/cmd_vel',
        }],
    )

    # Controller configuration
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, controllers_yaml_path],
    #     # remappings=[
    #     #     ("robot_description", "/piper/robot_description"),
    #     # ],
    #     output="screen",
    # )
    # load_controllers = []
    # control_mode = LaunchConfiguration('control_mode').perform(context).lower()
    # controllers = []
    # Design decision: spawn only one arm command controller to avoid mixed-command behavior.
    # if control_mode == 'position':
    #     controllers += ["arm_controller"]
    # else:
    #     controllers += ["arm_velocity_controller"]
    # if use_gripper.lower() == 'true':
    #     controllers += ["gripper_controller"]
    # for controller in controllers:
    #     load_controllers += [
    #         TimerAction(
    #         period=0.5,
    #         actions=[
    #             Node(
    #                 package="controller_manager",
    #                 executable="spawner.py",
    #                 arguments=[
    #                     controller,
    #                     "--controller-manager", "/controller_manager"
    #                 ],
    #             )
    #         ]
    #     )
    #     ]
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[robot_description],
    #     # namespace='piper'
    # )
    # run keyboard rel_move in a separate terminal

    jacobian_velctrl = Node(
        package="realtime_servo",
        executable="jacobian_velctrl_node",
        output="screen",
        parameters=[
            robot_description,
            {
                'base_link': LaunchConfiguration('jacobian_base_link'),
                'ee_link': LaunchConfiguration('jacobian_ee_link'),
                'joint_states_topic': LaunchConfiguration('jacobian_joint_states_topic'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'joint_limits_yaml': os.path.join(
                    get_package_share_directory('interbotix_xsarm_moveit'),
                    'config/joint_limits/wx200_joint_limits.yaml'
                ),
                'output_mode': LaunchConfiguration('control_mode'),
                'joint_position_command_topic': '/wx200/arm_controller/joint_trajectory',
                'joint_velocity_command_topic': '/wx200/arm_velocity_controller/commands',
                'velocity_command_topic': '/wx200/cmd_vel',
                'alpha': 0.8, # LPF coefficient for velocity smoothing, between [0, 1). 0 means no smoothing (raw Jacobian output), while closer to 1 means more smoothing
                'use_damped_pseudoinverse': False,
                'command_timeout_sec': 0.1,

            },
        ],
    )

    return [
        # robot_state_publisher_node,
        ## IMPORTANT! OVERRIDE JOINT STATE BROADCASTER. Solves random joint states order issue: https://github.com/ros-controls/ros2_controllers/issues/159
        # xs_control_launch,
        # ros2_control_node,
        # TimerAction(period=3.0, actions=[arm_controller_spawner]),
        TimerAction(period=8.0, actions=[hardware_launch]),
        move_group_node,
        position_server_node,
        cbf_filter_node,
        # *load_controllers,c
        TimerAction(period=14.0, actions=[jacobian_velctrl]),
        rviz_node,
    ]

def generate_launch_description():
    
    opfunc = OpaqueFunction(function=launch_setup)
    launch_args = generate_launch_args()
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld