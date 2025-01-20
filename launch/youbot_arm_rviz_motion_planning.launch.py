import os
import yaml
import xacro

from typing import Dict, Text

from launch import LaunchDescription
from launch.substitution import Substitution
from launch.substitutions import PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


class Xacro(Substitution):
    """
    Substitution that processes a xacro file and returns the result as a string
    
    :param file_path: The path to the xacro file to process
    :param mappings: A dictionary of mappings to pass to xacro

    example:

      state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Xacro(
                    file_path=JoinLaunchSubstitutions(
                        [
                            FindPackageShare("my_robot_description"),
                            "urdf",
                            "my_robot.urdf.xacro",
                        ]
                    ),
                    mappings={
                        "thing_to_substitute": "substitution_value",
                        "another_thing_to_substitute": LaunchConfiguration("another_thing_to_substitute"),
                    },
                )
            }
        ],
    )
    """

    def __init__(self, file_path: SomeSubstitutionsType, mappings: Dict[str, SomeSubstitutionsType] = {}, verbose: bool = False):
        """Create a TemplateSubstitution."""
        super().__init__()
        self.__file_path = normalize_to_list_of_substitutions(file_path)
        self.__mappings = {key: normalize_to_list_of_substitutions(value) for key, value in mappings.items()}
        self.__verbose = verbose
    
    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"Xacro: {self.__file_path}"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the string with values substituted."""

        file_path = perform_substitutions(context, self.__file_path)
        mappings = {key: perform_substitutions(context, value) for key, value in self.__mappings.items()}

        if self.__verbose:
            print(f"xacro file_path: {file_path}")
            print(f"xacro mappings: {mappings}")
        document = xacro.process_file(file_path, mappings=mappings)
        document_string = document.toprettyxml(indent="  ")

        if self.__verbose:
            print(f"xacro result: {document_string}")
        return document_string



def generate_launch_description():
    print(FindPackageShare("youbot_ros2").find("youbot_ros2"))

    # load URDF via xacro
    robot_description = {"robot_description": ParameterValue(
        Xacro(
            file_path=PathJoinSubstitution(
                [
                    FindPackageShare("youbot_ros2"),
                    "description",
                    "robots",
                    "youbot_arm_only.urdf.xacro",
                ]
            )
        ),
    value_type=str)}

    # load SRDF via xacro
    robot_description_semantic = {"robot_description_semantic": ParameterValue(
        Xacro(
            file_path=PathJoinSubstitution(
                [
                    FindPackageShare("youbot_ros2"),
                    "config",
                    "youbot_arm.srdf",
                ]
            )
        ),
    value_type=str)}

    # load kinematics
    robot_description_kinematics = { "robot_description_kinematics": load_yaml("youbot_ros2", "config/kinematics.yaml") }



#    rviz_config_file = PathJoinSubstitution(
#        [FindPackageShare("youbot_ros2"), "config", "moveit.rviz"]
#    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
#        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],
    )

    nodes = [
        rviz_node,
    ]

    return LaunchDescription(nodes)
