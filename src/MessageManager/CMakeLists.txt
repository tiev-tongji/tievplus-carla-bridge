cmake_minimum_required (VERSION 3.0)

# compile library
add_library (msg_manager "MessageManager.cpp")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_definitions(msg_manager PUBLIC "USE_ZCM")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_definitions(msg_manager PUBLIC "USE_LCM")
endif()

# link
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(msg_manager zcm Geographic carla_client
                          Detour DetourCrowd boost_filesystem
                          Recast rpc png jpeg tiff)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(msg_manager lcm ${GeographicLib_LIBRARIES} carla_client
                          Detour DetourCrowd libboost_filesystem-vc141-mt-x64-1_72
                          Recast rpc zlib libpng shlwapi)
endif()