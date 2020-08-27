# CMakeLists.txt: MessageManager
cmake_minimum_required (VERSION 3.0)

# compiler options
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# add search directories
find_package(GeographicLib)
include_directories (${CMAKE_SOURCE_DIR})
include_directories (${GeographicLib_INCLUDE_DIRS})
include_directories (${CMAKE_SOURCE_DIR}/msg/include_lcm)
include_directories (${CMAKE_SOURCE_DIR}/LibCarla/include)
include_directories (${CMAKE_SOURCE_DIR}/LibCarla/include/system)
link_directories (${CMAKE_SOURCE_DIR}/LibCarla/lib)
link_directories (${CMAKE_SOURCE_DIR}/lcm/lib)

# compile library
add_library (msg_manager_new "MessageManager.cpp")

# link
target_link_libraries(msg_manager_new lcm ${GeographicLib_LIBRARIES} carla_client
                      Detour DetourCrowd libboost_filesystem-vc141-mt-x64-1_72
                      Recast rpc zlib libpng shlwapi)