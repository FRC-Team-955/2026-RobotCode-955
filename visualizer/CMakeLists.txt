cmake_minimum_required(VERSION 3.15)
project(visualizer)

find_package(ntcore REQUIRED)
find_package(raylib 3.0 REQUIRED) # Requires at least version 3.0
find_package(wpimath REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} ntcore raylib wpimath)

# Checks if OSX and links appropriate frameworks (only required on MacOS)
if (APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework IOKit")
    target_link_libraries(${PROJECT_NAME} "-framework Cocoa")
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()
