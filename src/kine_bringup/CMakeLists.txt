cmake_minimum_required(VERSION 3.24)
project(kine_bringup)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
