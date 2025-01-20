from sys import prefix
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(robot_name="youbot_arm", package_name="youbot_ros2")
        .robot_description(file_path="description/robots/youbot_arm_only.urdf.xacro")
        .robot_description_semantic(file_path="config/youbot_arm.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            # pilz_industrial_motion_planner requires pilz_cartesian_limits.yaml config file
            # pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
            # pipelines=["ompl", "chomp", "stomp"]
            pipelines=["ompl", "stomp"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using real as hardware
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("youbot_ros2"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        exec_name="controller_manager",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
        prefix=[
            ExecutableInPackage(
                package="soem_ethercat_grant",
                executable="soem_ethercat_grant",
            ),
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    youbot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "youbot_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    # Delay start of youbot_arm_controller after `joint_state_broadcaster`
    delay_youbot_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                youbot_arm_controller_spawner,
            ],
        )
    )


    return LaunchDescription(
        [
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            # youbot_arm_controller_spawner,
            delay_youbot_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )