cmake_minimum_required(VERSION 3.5)
project(rmul24_gazebo_simulator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    config
    launch
    rviz
    resource

    DESTINATION share/${PROJECT_NAME}/
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

ament_package()
