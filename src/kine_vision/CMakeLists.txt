cmake_minimum_required(VERSION 3.24)
project(kine_vision)

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

find_package(OpenCV REQUIRED)

add_executable(kine_vision kine_vision.cpp)
target_include_directories(kine_vision PUBLIC include)
ament_target_dependencies(kine_vision rclcpp ament_index_cpp std_msgs sensor_msgs)
target_link_libraries(kine_vision ${OpenCV_LIBS})


install(TARGETS
        kine_vision
        DESTINATION lib/${PROJECT_NAME})


ament_package()
