
cmake_minimum_required (VERSION 3.0)
project(tiev-sim)

set(SRC ${CMAKE_SOURCE_DIR}/src)
set(THIRD_PARTY ${CMAKE_SOURCE_DIR}/third_party)

# path
include_directories(${SRC})
include_directories(${THIRD_PARTY})
include_directories(${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include)
include_directories(${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include/system)
link_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/lib)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include_directories(${THIRD_PARTY}/msg/include_zcm)
    include_directories("/usr/local/include")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    include_directories(${THIRD_PARTY}/msg/include_lcm)
    link_directories(${THIRD_PARTY}/lcm/lib)
    find_package(GeographicLib)
    include_directories(${GeographicLib_INCLUDE_DIRS})
endif()

# add sub-target directories
add_subdirectory(${SRC}/VehicleController)
add_subdirectory(${SRC}/MessageManager)
add_subdirectory(${SRC}/MyWorld)