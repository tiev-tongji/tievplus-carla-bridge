cmake_minimum_required (VERSION 3.0)

# if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
#     target_compile_definitions(msg_manager PUBLIC "USE_ZCM")
# endif()
# if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
#     target_compile_definitions(msg_manager PUBLIC "USE_LCM")
# endif()

# compile library
add_library (myworld "MyWorld.cpp" "MyPlayer.cpp")