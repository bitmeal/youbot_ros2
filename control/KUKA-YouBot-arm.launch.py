from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():
    print(FindPackageShare("youbot_ros2").find("youbot_ros2"))

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("youbot_ros2"),
                    "description",
                    "robots",
                    "youbot_arm_only.urdf.xacro",
                ]
            ),
        ]
    )

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ros2_control_demo_example_1"),
    #         "config",
    #         "rrbot_controllers.yaml",
    #     ]
    # )

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="cat")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 "KUKA-YouBot-arm.urdf",
    #             ]
    #         ),
    #     ]
    # )

    robot_controllers = PathJoinSubstitution(
        [
            "KUKA-YouBot-arm_controllers.yaml",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        exec_name="controller_manager",
        parameters=[robot_description, robot_controllers],
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

    # control_node = Node(
    #     package="soem_ethercat_grant",
    #     executable="soem_ethercat_grant",
    #     exec_name="controller_manager",
    #     parameters=[robot_description, robot_controllers],
    #     output="both",
    #     arguments=[
    #         ExecutableInPackage(
    #             executable="ros2_control_node",
    #             package="controller_manager",
    #         ),
    #         '--ros-args',
    #         '--log-level', 'info'
    #     ]
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            # "--controller-manager",
            # "/controller_manager",
            # "--controller-manager-timeout=15"
        ]
    )

    robot_controller_spawner_velocity = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller"] # , "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner_position = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"] # , "--controller-manager", "/controller_manager"],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                robot_controller_spawner_velocity,
                # robot_controller_spawner_position
            ],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # robot_controller_spawner_velocity,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)