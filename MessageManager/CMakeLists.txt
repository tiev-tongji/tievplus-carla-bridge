# CMakeLists.txt: MessageManager
cmake_minimum_required (VERSION 3.0)

# compiler options
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# add search directories
include_directories ("/usr/local/include/")
include_directories (${CMAKE_SOURCE_DIR}/msg/include)
include_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include)
include_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include/system)
link_directories (${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/lib)
# compile library
add_library (msg_manager "MessageManager.cpp")

# link
target_link_libraries(msg_manager zcm Geographic carla_client
                      Detour DetourCrowd boost_filesystem
                      Recast rpc png jpeg tiff)