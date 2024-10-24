cmake_minimum_required(VERSION 3.8)
project(blackbox)

add_definitions(-DBLACKBOX_PARAM_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/config")

find_package(tutrc_ament REQUIRED)

tutrc_ament_init()

file(GLOB_RECURSE CXX_SOURCES "src/*.cpp")
tutrc_ament_add_library(${PROJECT_NAME} SHARED ${CXX_SOURCES})

tutrc_ament_final()
