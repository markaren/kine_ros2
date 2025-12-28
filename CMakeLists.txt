cmake_minimum_required(VERSION 3.16)
project(ros2_ws LANGUAGES C CXX)


# This file exists ONLY so IDEs (CLion) have a single entry point.
# colcon still drives the actual build.

if (WIN32)
    # this is only to help CLion find OpenSSL on Windows
    find_package(OpenSSL QUIET)
    if (NOT OpenSSL_FOUND)
        message(STATUS "OpenSSL not found, trying to locate in pixi environment...")
        set(OPENSSL_ROOT_DIR "C:/pixi_ws/.pixi/envs/default/Library")
        set(OPENSSL_USE_STATIC_LIBS FALSE)
    endif ()
endif ()

find_package(ament_cmake REQUIRED)

# Add each package as a subdirectory
add_subdirectory(src/kine)
add_subdirectory(src/interactive_goal)

# ---- colcon build target ----
add_custom_target(colcon_build
        COMMAND colcon build --packages-select kine interactive_goal --base-paths src --merge-install --event-handlers console_direct+
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        USES_TERMINAL
        COMMENT "Running colcon build --merge-install"
)
