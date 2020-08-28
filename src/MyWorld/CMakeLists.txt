cmake_minimum_required (VERSION 3.0)

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