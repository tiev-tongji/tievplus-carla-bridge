
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
    add_definitions(-DUSE_ZCM)
    include_directories(${THIRD_PARTY}/msg/include_zcm)
    include_directories("/usr/local/include")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_definitions(-DUSE_LCM)
    include_directories(${THIRD_PARTY}/msg/include_lcm)
    link_directories(${THIRD_PARTY}/lcm/lib)
    find_package(GeographicLib)
    include_directories(${GeographicLib_INCLUDE_DIRS})
endif()

# add sub-target directories
add_subdirectory(${SRC}/VehicleController)
add_subdirectory(${SRC}/MessageManager)
add_subdirectory(${SRC}/MyWorld)

# add executable
add_executable(tiev-sim "tiev-sim.cpp")

# link
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(CARLA_LIB carla_client Detour DetourCrowd boost_filesystem Recast rpc png jpeg tiff)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(CARLA_LIB carla_client Detour DetourCrowd libboost_filesystem-vc141-mt-x64-1_72 Recast rpc zlib libpng shlwapi)
endif()

target_link_libraries(tiev-sim msg_manager myworld tievplus_controller CARLA_LIB)