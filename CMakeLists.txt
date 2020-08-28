
cmake_minimum_required (VERSION 3.0)
project(tiev-sim)

set(SRC ${CMAKE_SOURCE_DIR}/src)
set(THIRD_PARTY ${CMAKE_SOURCE_DIR}/src/third_party)

# path


# add sub-target directories
add_subdirectory(${SRC}/PIDController)
add_subdirectory (${SRC}/MessageManager)

# compile
add_executable (tiev-sim "tiev-sim.cpp")

# link
#target_link_libraries (tiev-sim pid_controller msg_manager sfml-graphics sfml-window sfml-system)
target_link_libraries (tiev-sim pid_controller msg_manager)