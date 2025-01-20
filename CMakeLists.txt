cmake_minimum_required(VERSION 3.12)
project(youbot_ros2)

# find dependencies
find_package(ament_cmake REQUIRED)

# install files
install(
    DIRECTORY control/
    DESTINATION share/${PROJECT_NAME}/control
)
install(
    DIRECTORY description/meshes/
    DESTINATION share/${PROJECT_NAME}/description/meshes
)
install(
    DIRECTORY description/robots/
    DESTINATION share/${PROJECT_NAME}/description/robots
)
install(
    DIRECTORY description/urdf/
    DESTINATION share/${PROJECT_NAME}/description/urdf
)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

## EXPORTS
ament_export_dependencies(
    soem_driver_ros2
)
ament_package()
