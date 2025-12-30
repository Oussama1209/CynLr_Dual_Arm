# Copyright (c) 2023
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml, xacro


def generate_launch_description():

    package_path = get_package_share_directory("multi_robot_arm")

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value="true", description="Use simulator time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world = LaunchConfiguration("world")
    declare_world_path = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(package_path, "worlds", "empty.world"),
        description="Full path to world model file to load",
    )

    # This LaunchArgument is not strictly needed for per-robot type,
    # but we keep it for completeness
    declare_robot_type = DeclareLaunchArgument(
        name="robot_type", default_value="panda", description="Default robot type"
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            "-s",
            "libgazebo_ros_factory.so",
            "-s",
            "libgazebo_ros_init.so",
            world,
        ],
        output="screen",
    )
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")

    urdf_file = os.path.join(package_path, "urdf", "grasp_cube.urdf.xacro")

    ld = LaunchDescription()
    ld.add_action(declare_world_path)
    ld.add_action(declare_robot_type)
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    # --- HERE we define which robots we want ---
    # 1 Panda + 1 UR5
    robots = [
        {"type": "ur5",   "name": "arm1", "x_pose": "0.0",  "y_pose": "1.5",  "Y": "-1.57079632679", "use_camera": "false"},
        {"type": "panda", "name": "arm5", "x_pose": "-10000.5", "y_pose": "0.0", "Y": "0.0", "use_camera": "false"},
        {"type": "panda", "name": "arm2", "x_pose": "0.0",  "y_pose": "-1.5",  "Y": "1.57079632679", "use_camera": "true"},
    ]

    # Multiple ARMs in gazebo must be spawned in a serial fashion due to
    # a global namespace dependency introduced by ros2_control.
    robot_final_action = None
    for robot in robots:
        robot_final_action = spawn_robot(
            ld,
            robot["type"],
            robot["name"],
            use_sim_time,
            robot["x_pose"],
            robot["y_pose"],
            robot["Y"],
            robot["use_camera"],
            robot_final_action,
        )

    # Command 1 message (pose A)
    pose_a_msg = (
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, "
        "joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], "
        "points: [{positions: [0.6,0.1,0.0,0.0,0.0,0.0], velocities: [], accelerations: [], effort: [], "
        "time_from_start: {sec: 1, nanosec: 0}}]}"
    )

    # Command 3 message (pose B)
    pose_b_msg = (
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, "
        "joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'], "
        "points: [{positions: [0.0,-2.3,1.7,0.6,1.57079632679,0.0], velocities: [], accelerations: [], effort: [], "
        "time_from_start: {sec: 1, nanosec: 0}}]}"
    )

    # 1) Publish pose A
    cmd_pose_a = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/arm1/arm_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            pose_a_msg,
        ],
        output="screen",
    )

    # 2) Attach
    cmd_attach = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/ATTACHLINK",
            "linkattacher_msgs/srv/AttachLink",
            "{model1_name: 'arm1', link1_name: 'wrist_3_link', model2_name: 'grasp_cube', link2_name: 'link'}",
        ],
        output="screen",
    )

    # 3) Publish pose B
    cmd_pose_b = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/arm1/arm_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            pose_b_msg,
        ],
        output="screen",
    )

    # 4) MoveL on arm1
    cmd_arm1_movel = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/arm1/MoveL",
            "ros2_data/action/MoveL",
            "{movex: 0.2, movey: 0.0, movez: 0.0, speed: 1.0}",
        ],
        output="screen",
    )

    # 5) MoveL on arm2
    cmd_arm2_movel = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/arm2/MoveL",
            "ros2_data/action/MoveL",
            "{movex: 0.4, movey: 0.0, movez: -0.3, speed: 1.0}",
        ],
        output="screen",
    )

    # 6) Enable tracking service on arm2
    cmd_arm2_enable_tracking = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/arm2/enable_tracking",
            "std_srvs/srv/SetBool",
            "{data: true}",
        ],
        output="screen",
    )

    start_after_10s = TimerAction(period=10.0, actions=[cmd_pose_a])
    attach_after_pose_a = TimerAction(period=15.0, actions=[cmd_attach])
    finish_after_10s = TimerAction(period=20.0, actions=[cmd_pose_b])

    arm1_movel_after = TimerAction(period=25.0, actions=[cmd_arm1_movel])
    arm2_movel_after = TimerAction(period=30.0, actions=[cmd_arm2_movel])

    enable_tracking_after = TimerAction(period=40.0, actions=[cmd_arm2_enable_tracking])

    ld.add_action(start_after_10s)
    ld.add_action(attach_after_pose_a)
    ld.add_action(finish_after_10s)

    ld.add_action(arm1_movel_after)
    ld.add_action(arm2_movel_after)
    
    ld.add_action(enable_tracking_after)

    return ld


def spawn_robot(
    ld, robot_type, robot_name, use_sim_time, x, y, Y, use_camera, previous_final_action=None
):

    package_path = get_package_share_directory("multi_robot_arm")
    namespace = "/" + robot_name

    # ------------------------------------------------------------------
    # Robot-type-specific configuration (paths, joint names, pedestal, etc.)
    # ------------------------------------------------------------------
    if robot_type == "panda":
        controllers_relpath = "config/panda/ros2_controllers.yaml"
        xacro_relpath = "urdf/panda/urdf/panda.urdf.xacro"
        kinematics_relpath = "config/panda/kinematics.yaml"
        srdf_relpath = "config/panda/panda.srdf"
        ompl_relpath = "config/panda/ompl_planning.yaml"
        moveit_ctrl_mgr_relpath = "config/panda/moveit_controller_manager.yaml"
        joint_limits_relpath = None  # no separate joint_limits yaml for panda
        pedestal_height = "1.0"
        planning_group = "panda_arm"
        use_camera = use_camera

        initial_trajectory_message = """ {
            'header': {
                'stamp': {
                    'sec': 0,
                    'nanosec': 0
                },
                'frame_id': ''
            },
            'joint_names': [
                'panda_joint1',
                'panda_joint2',
                'panda_joint3',
                'panda_joint4',
                'panda_joint5',
                'panda_joint6',
                'panda_joint7'
            ],
            'points': [
                {
                    'positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    'velocities': [],
                    'accelerations': [],
                    'effort': [],
                    'time_from_start': {
                        'sec': 1,
                        'nanosec': 0
                    }
                }
            ]
        }"""

    elif robot_type == "ur5":
        controllers_relpath = f"config/ur/{robot_type}/ros_controllers_robot.yaml"
        xacro_relpath = "urdf/ur/ur5/ur_urdf.xacro"
        kinematics_relpath = f"config/ur/{robot_type}/kinematics.yaml"
        srdf_relpath = f"config/ur/{robot_type}/robot.srdf"
        ompl_relpath = f"config/ur/{robot_type}/ompl_planning.yaml"
        moveit_ctrl_mgr_relpath = f"config/ur/{robot_type}/moveit_controller_manager.yaml"
        joint_limits_relpath = f"config/ur/{robot_type}/joint_limits_planning.yaml"
        pedestal_height = "0.1"
        planning_group = "arm"
        use_camera = use_camera

        initial_trajectory_message = """ {
            'header': {
                'stamp': {
                    'sec': 0,
                    'nanosec': 0
                },
                'frame_id': ''
            },
            'joint_names': [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ],
            'points': [
                {
                    'positions': [0.0, -2.3, 1.7, 0.6, 1.57079632679, 0.0],
                    'velocities': [],
                    'accelerations': [],
                    'effort': [],
                    'time_from_start': {
                        'sec': 1,
                        'nanosec': 0
                    }
                }
            ]
        }"""

    else:
        raise RuntimeError(f"Unsupported robot_type '{robot_type}'")

    # ------------------------------------------------------------------
    # ros2_control YAML (RewrittenYaml for per-namespace controller manager)
    # ------------------------------------------------------------------
    param_substitutions = {"use_sim_time": use_sim_time}
    controllers_yaml_abs = os.path.join(package_path, controllers_relpath)

    configured_params = RewrittenYaml(
        source_file=controllers_yaml_abs,
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    context = LaunchContext()
    controller_paramfile = configured_params.perform(context)

    # ------------------------------------------------------------------
    # Build URDF from xacro with correct simulation_controllers & namespace
    # ------------------------------------------------------------------
    xacro_path = os.path.join(package_path, xacro_relpath)

    robot_doc = xacro.process_file(
        xacro_path,
        mappings={
            "name": robot_name,
            "namespace": namespace,
            "sim_gazebo": "1",
            "simulation_controllers": controller_paramfile,
            "safety_limits": "true",
            "prefix": "",
            "pedestal_height": pedestal_height,
            "use_camera": use_camera,
        },
    )

    robot_urdf = robot_doc.toprettyxml(indent="  ")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_params = {"robot_description": robot_urdf, "use_sim_time": use_sim_time}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        output="screen",
        remappings=remappings,
        parameters=[robot_params],
    )

    robot_description = {"robot_description": robot_urdf}

    # ------------------------------------------------------------------
    # MoveIt parameters (shared pattern, robot-type-specific YAMLs)
    # ------------------------------------------------------------------
    kinematics_yaml = load_yaml(package_path, kinematics_relpath)

    robot_description_semantic_config = load_file(package_path, srdf_relpath)
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }

    ompl_planning_yaml = load_yaml(package_path, ompl_relpath)
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        package_path, moveit_ctrl_mgr_relpath
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": True,
        "trajectory_execution.controller_connection_timeout": 30.0,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "default_planning_pipeline": "ESTkConfigDefault",
        "use_sim_time": use_sim_time,
    }

    pipeline_names = {"pipeline_names": ["ompl"]}

    planning_pipelines = {
        "planning_pipelines": pipeline_names,
        "default_planning_pipeline": "ompl",
    }

    # Optional joint limits for UR5
    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        planning_pipelines,
        {"planning_plugin": "ompl", "use_sim_time": use_sim_time},
    ]

    if joint_limits_relpath is not None:
        joint_limits_yaml = load_yaml(package_path, joint_limits_relpath)
        joint_limits = {"robot_description_planning": joint_limits_yaml}
        move_group_parameters.append(joint_limits)

    # Start the actual move_group node/action server
    robot_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=move_group_parameters,
        remappings=remappings,
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ------------------------------------------------------------------
    # Gazebo spawning + ros2_control controller loading sequence
    # ------------------------------------------------------------------
    ros_distro = os.environ.get("ROS_DISTRO")

    controller_run_state = "active"
    if ros_distro == "foxy":
        controller_run_state = "start"

    robot_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            namespace + "/robot_description",
            "-entity",
            robot_name,
            "-robot_namespace",
            namespace,
            "-x",
            x,
            "-y",
            y,
            "-z",
            "0.0",
            "-Y",
            Y,
            "-unpause",
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "joint_state_broadcaster",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    load_arm_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "arm_controller",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    # Initial joint position for each robot (type-specific message defined above)
    set_initial_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/" + robot_name + "/arm_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            initial_trajectory_message,
        ],
        output="screen",
    )

    set_initial_pose_movel = ExecuteProcess(
        cmd=[
            "ros2",
            "action",
            "send_goal",
            "-f",
            f"/{robot_name}/MoveL",
            "ros2_data/action/MoveL",
            "{movex: 0.20, movey: 0.0, movez: -0.20, speed: 1.0}",
        ],
        output="screen",
    )

    # Serial spawning dependency chain
    if previous_final_action is not None:
        spawn_entity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_final_action,
                on_exit=[robot_spawn_entity],
            )
        )
    else:
        spawn_entity = robot_spawn_entity

    state_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_arm_trajectory_controller],
        )
    )

    set_initial_pose_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_trajectory_controller,
            on_exit=[set_initial_pose],
        )
    )

    set_initial_pose_movel_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_trajectory_controller,
            on_exit=[set_initial_pose_movel],
        )
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(robot_move_group_node)
    ld.add_action(spawn_entity)
    ld.add_action(state_controller_event)
    ld.add_action(arm_controller_event)
    ld.add_action(set_initial_pose_event)

    moveL_action_node = Node(
        package="ros2_actions",
        executable="moveL_action",
        namespace=namespace,
        name=f"{robot_name}_moveL_action",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": use_sim_time},    # <-- important
            {"ROB_PARAM": planning_group},     # e.g. "panda_arm" or "ur5_arm"
        ],
        remappings=remappings,
    )

    moveL_circle_action_node = Node(
        package="ros2_actions",
        executable="movel_circle_action",   # must match CMake
        namespace=namespace,
        name=f"{robot_name}_moveL_circle_action",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": use_sim_time},
            {"ROB_PARAM": planning_group},   # "panda_arm" / "arm"
            {"num_points": 20},
        ],
        remappings=remappings,
    )

    # MoveR ACTION:
    moveR_interface = Node(
        name=f"{robot_name}_moveR_action",
        package="ros2_actions",
        executable="moveR_action",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description, 
            robot_description_semantic, 
            kinematics_yaml, 
            {"use_sim_time": use_sim_time},
            {"ROB_PARAM": planning_group},
        ],
        remappings=remappings,    
    )

    ld.add_action(moveL_action_node)
    ld.add_action(moveL_circle_action_node)
    ld.add_action(moveR_interface)

    if namespace == '/arm2':
        follow_box = Node(
            package='ros2_actions',
            executable='yz_plane_tracker',
            namespace=namespace, 
            name='arm2_yz_plane_tracker',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_yaml,
                {
                    'ROB_PARAM': 'panda_arm',
                    # tracker-specific tuning
                    'gain_y': 0.5,
                    'gain_z': 0.5,
                    'max_step': 0.1,
                    'deadband': 0.05,
                    'eef_step': 0.05,
                    'control_rate': 10.0,
                    'vel_scale': 0.5,
                    'acc_scale': 0.5,
                    'segment_time': 0.5,
                },
            ],
        )
        ld.add_action(follow_box)

    return load_arm_trajectory_controller


def load_file(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
