cmake_minimum_required(VERSION 3.15)
project(dxProjectAssembleLegos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find our FindMuJoCo.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(MuJoCo REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

add_executable(dxProjectAssembleLegos main.cpp
    dxRobotViewerFactory.cpp
    dxRobotViewerFactory.h
    dxRobotSimulator.cpp
    dxRobotSimulator.h)

target_link_libraries(dxProjectAssembleLegos PRIVATE
  MuJoCo::mujoco
  glfw
  OpenGL::GL
)


