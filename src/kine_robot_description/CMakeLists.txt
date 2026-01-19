cmake_minimum_required(VERSION 3.24)
project(kine_robot_description)

find_package(ament_cmake REQUIRED)


install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
