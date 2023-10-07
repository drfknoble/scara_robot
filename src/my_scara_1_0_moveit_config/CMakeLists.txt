cmake_minimum_required(VERSION 3.10.2)
project(my_scara_1_0_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config rviz
        DESTINATION share/${PROJECT_NAME})
