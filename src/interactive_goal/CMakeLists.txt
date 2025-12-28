cmake_minimum_required(VERSION 3.15)
project(interactive_goal)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(interactive_goal_node interactive_goal_node.cpp)
ament_target_dependencies(interactive_goal_node rclcpp std_msgs geometry_msgs visualization_msgs interactive_markers)


install(TARGETS
        interactive_goal_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
