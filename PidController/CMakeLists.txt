cmake_minimum_required(VERSION 3.0)

# search dir
include_directories(${THIRD_PARTY})

# compile library
add_library(pid_controller "PidController.cpp")