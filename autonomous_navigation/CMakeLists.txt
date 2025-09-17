cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(autonomous_navigation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

find_package(Threads REQUIRED)

# Includes for the path planning server - only include when not building on the bot
if(${MACHINE_TYPE} STREQUAL "LAPTOP")
  find_package(Boost 1.54.0 COMPONENTS system thread coroutine context REQUIRED)
  find_package(OpenSSL REQUIRED)
  set(SIMPLE_WS_SERVER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()


# If we're building for the Omnibot, find LCM.
if(${MACHINE_TYPE} STREQUAL "OMNI")
  find_package(lcm REQUIRED)
  include(${LCM_USE_FILE})
endif()

# Server executable. Only build if we are not on the Omnibot
if(${MACHINE_TYPE} STREQUAL "LAPTOP")
  add_executable(nav_app_server src/web_server.cpp
    src/graph_search/graph_search.cpp
    src/potential_field/distance_transform.cpp
    src/potential_field/potential_field.cpp
    src/potential_field/local_search.cpp
    src/utils/graph_utils.cpp
  )
  target_link_libraries(nav_app_server
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${OPENSSL_LIBRARIES}
  )
  target_include_directories(nav_app_server PRIVATE
    include
    ${OPENSSL_INCLUDE_DIR}
    ${SIMPLE_WS_SERVER_INCLUDE_DIR}
  )
endif()

# If we're building for the Omnibot, build the LCM server.
if(${MACHINE_TYPE} STREQUAL "OMNI")
  add_executable(robot_plan_path src/robot_plan_path.cpp
    src/potential_field/distance_transform.cpp
    src/graph_search/graph_search.cpp
    src/utils/graph_utils.cpp
  )
  target_link_libraries(robot_plan_path
    lcm
    omnibot_msgs-cpp
  )
  target_include_directories(robot_plan_path PRIVATE
    include
  )

  add_executable(robot_potential_field src/robot_potential_field.cpp
    src/utils/graph_utils.cpp
    src/potential_field/distance_transform.cpp
    src/potential_field/potential_field.cpp
    src/potential_field/local_search.cpp
  )
  target_link_libraries(robot_potential_field
    lcm
    omnibot_msgs-cpp
  )
  target_include_directories(robot_potential_field PRIVATE
    include
  )
endif()

# Test executables.
add_executable(test_graph src/tests/test_graph.cpp
  src/utils/graph_utils.cpp
)
target_link_libraries(test_graph
)
target_include_directories(test_graph PRIVATE
  include
)

if (CMAKE_BUILD_TYPE MATCHES Test)
endif()
