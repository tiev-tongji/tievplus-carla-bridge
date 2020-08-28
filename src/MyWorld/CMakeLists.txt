cmake_minimum_required (VERSION 3.0)

# add search directories
include_directories(${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include)
include_directories(${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/include/system)
link_directories(${CMAKE_SOURCE_DIR}/../PythonAPI/carla/dependencies/lib)

# compile library
add_library (myworld "MyWorld.cpp")

# link
target_link_libraries(myworld carla_client
                      Detour DetourCrowd boost_filesystem
                      Recast rpc png jpeg tiff)