cmake_minimum_required(VERSION 3.10.2)

project(my_scara_support)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
