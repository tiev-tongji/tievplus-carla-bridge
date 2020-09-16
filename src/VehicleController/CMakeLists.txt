cmake_minimum_required(VERSION 3.0)

# search dir
include_directories(${THIRD_PARTY})

# compile library
add_library(pid_controller "PidControllerBase.cpp")
add_library(lon_controller "LongitudinalController.cpp")
add_library(tievplus_controller "TievplusController.cpp")

# link
target_link_libraries(tievplus_controller lon_controller pid_controller)