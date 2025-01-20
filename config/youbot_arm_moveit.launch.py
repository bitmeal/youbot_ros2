import os
from sys import prefix
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
# from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# def load_file(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return file.read()
#     # parent of IOError, OSError *and* WindowsError where available
#     except EnvironmentError:
#         return None


# def load_yaml(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return yaml.safe_load(file)
#     # parent of IOError, OSError *and* WindowsError where available
#     except EnvironmentError:
#         return None

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

    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
    #     .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
    #     .to_moveit_configs()
    # )

    # moveit_config = (
    #     MoveItConfigsBuilder("moveit_resources_panda")
    #     .robot_description(
    #         file_path="config/panda.urdf.xacro",
    #         mappings={
    #             "ros2_control_hardware_type": LaunchConfiguration(
    #                 "ros2_control_hardware_type"
    #             )
    #         },
    #     )
    #     .robot_description_semantic(file_path="config/panda.srdf")
    #     .planning_scene_monitor(
    #         publish_robot_description=True, publish_robot_description_semantic=True
    #     )
    #     .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    #     .planning_pipelines(
    #         pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
    #     )
    #     .to_moveit_configs()
    # )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=[
            '--ros-args',
            '--log-level',
            'info'
        ],
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
    # ros2_controllers_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare("youbot_ros2"),
    #         "control",
    #         "KUKA-YouBot-arm_controllers.yaml",
    #     ]
    # )
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

            # ' '.join([' ',
            #     '/home/edgelord/DrMemory-Linux-2.6.0/bin64/drmemory',
            #     '--',
            # ])

            # ' '.join([' ',
            #     'valgrind',
            #     # '--trace-children=yes',
            #     '--tool=memcheck', '--leak-check=yes',
            #     # '--tool=helgrind',
            #     # '--tool=drd',
            #     # '--log-file="valgrind-%p-%n.dat"',
            # ]),
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

    # configure controller, but do not activate
    youbot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "youbot_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive"
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