from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("ros2_control_demo_example_1"),
    #                 "urdf",
    #                 "rrbot.urdf.xacro",
    #             ]
    #         ),
    #     ]
    # )
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ros2_control_demo_example_1"),
    #         "config",
    #         "rrbot_controllers.yaml",
    #     ]
    # )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]),
            " ",
            PathJoinSubstitution(
                [
                    "TMCM-1610_single_joint_mechanism.urdf",
                ]
            ),
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            "TMCM-1610_single_joint_mechanism_controllers.yaml",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, robot_controllers],
    #     output="both",
    #     arguments=['--ros-args', '--log-level', 'info']
    # )

    control_node = Node(
        package="soem_ethercat_grant",
        executable="soem_ethercat_grant",
        parameters=[robot_description, robot_controllers],
        output="both",
        arguments=[
            ExecutableInPackage(
                executable="ros2_control_node",
                package="controller_manager",
            ),
            '--ros-args',
            '--log-level', 'info'
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"] # , "--controller-manager", "/controller_manager"],
    )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_velocity_controller"] # , "--controller-manager", "/controller_manager"],
    # )

    # # robot_state_pub_node = Node(
    # #     package="robot_state_publisher",
    # #     executable="robot_state_publisher",
    # #     output="both",
    # #     parameters=[robot_description],
    # # )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    nodes = [
        control_node,
        # robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)