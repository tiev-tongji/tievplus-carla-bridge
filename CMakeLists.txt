# CMakeLists.txt: tiev-sim
cmake_minimum_required (VERSION 3.0)
project(tiev-sim)

# compiler options
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# search path
include_directories (${CMAKE_SOURCE_DIR})
include_directories (${CMAKE_SOURCE_DIR}/MessageManager)
include_directories (${CMAKE_SOURCE_DIR}/PIDController)
include_directories (${CMAKE_SOURCE_DIR}/msg/include_lcm)
include_directories (${CMAKE_SOURCE_DIR}/LibCarla/include)
include_directories (${CMAKE_SOURCE_DIR}/LibCarla/include/system)
link_directories (${CMAKE_SOURCE_DIR}/LibCarla/lib)
link_directories (${CMAKE_SOURCE_DIR}/lcm/lib)

# add sub-target directories
add_subdirectory ("${CMAKE_SOURCE_DIR}/PIDController")
add_subdirectory ("${CMAKE_SOURCE_DIR}/MessageManager")

# compile
add_executable (tiev-sim "tiev-sim.cpp")
add_executable (cpp-client-demo "demo.cpp")

# link
target_link_libraries (tiev-sim pid_controller msg_manager)
target_link_libraries (cpp-client-demo carla_client Detour DetourCrowd libboost_filesystem-vc141-mt-x64-1_72 Recast rpc libpng zlib shlwapi)