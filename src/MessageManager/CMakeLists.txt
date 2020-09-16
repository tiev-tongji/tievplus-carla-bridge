cmake_minimum_required (VERSION 3.0)

# compile library
add_library (msg_manager "MessageManager.cpp" "MessageManagerBase.cpp")
# if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
#     target_compile_definitions(msg_manager PUBLIC "USE_ZCM")
# endif()
# if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
#     target_compile_definitions(msg_manager PUBLIC "USE_LCM")
# endif()

# link
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(msg_manager zcm Geographic)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(msg_manager lcm ${GeographicLib_LIBRARIES})
endif()