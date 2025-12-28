cmake_minimum_required(VERSION 3.24)
project(kine)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include(FetchContent)
set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG cbac2a9f2f6dc9b0521e4d8cf2fe6a6200c662a4
)
FetchContent_MakeAvailable(threepp)

add_subdirectory(${threepp_SOURCE_DIR}/examples/external ${CMAKE_CURRENT_BINARY_DIR}/threepp_examples_external)


add_executable(kine_environment kine_environment.cpp)
ament_target_dependencies(kine_environment rclcpp ament_index_cpp std_msgs sensor_msgs)
target_link_libraries(kine_environment threepp::threepp imgui)
target_compile_definitions(kine_environment PRIVATE PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(kine_control kine_control.cpp)
ament_target_dependencies(kine_control rclcpp ament_index_cpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(kine_control threepp::threepp Eigen3::Eigen)

install(TARGETS
        kine_control
        kine_environment
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
