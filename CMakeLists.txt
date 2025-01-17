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
include_directories(${CMAKE_SOURCE_DIR}/common)
include_directories (${CMAKE_SOURCE_DIR}/MessageManager)
include_directories (${CMAKE_SOURCE_DIR}/PIDController)
include_directories (${CMAKE_SOURCE_DIR}/msg/include)
include_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include)
include_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include/system)
link_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/lib)

# add sub-target directories
add_subdirectory ("${CMAKE_SOURCE_DIR}/PIDController")
add_subdirectory ("${CMAKE_SOURCE_DIR}/MessageManager")
add_subdirectory ("${CMAKE_SOURCE_DIR}/common")

# compile
add_executable (tiev-sim "tiev-sim.cpp")
add_executable(tiev-sim-once "tiev-sim-once-plan.cpp")
add_executable (cpp-client-demo "demo.cpp")
add_executable (syncmode-demo "syncmode_demo.cpp")

# link
#target_link_libraries (tiev-sim pid_controller msg_manager sfml-graphics sfml-window sfml-system)
target_link_libraries (tiev-sim pid_controller msg_manager coordinate_converter nature)
target_link_libraries (tiev-sim-once pid_controller msg_manager_utm coordinate_converter nature)
target_link_libraries (syncmode-demo msg_manager)
target_link_libraries (cpp-client-demo carla_client Detour DetourCrowd boost_filesystem Recast rpc png jpeg tiff)