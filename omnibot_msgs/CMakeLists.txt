cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(omnibot_msgs)

find_package(lcm REQUIRED)
# find_package(PythonInterp)

include(${LCM_USE_FILE})
include(GenerateExportHeader)

# if(PYTHONINTERP_FOUND)
#   set(python_args PYTHON_SOURCES python_install_sources)
# endif()

lcm_wrap_types(
  C_EXPORT omnibot_msgs
  C_SOURCES c_sources
  C_HEADERS c_headers
  CPP_HEADERS cpp_headers
  # ${python_args}
  lcmtypes/occupancy_grid_t.lcm
  lcmtypes/omni_motor_command_t.lcm
  lcmtypes/pose_xyt_t.lcm
  lcmtypes/robot_path_t.lcm
)

lcm_add_library(omnibot_msgs C ${c_sources} ${c_headers})
generate_export_header(omnibot_msgs)
target_include_directories(omnibot_msgs INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(omnibot_msgs INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

lcm_add_library(omnibot_msgs-cpp CPP ${cpp_headers})
target_include_directories(omnibot_msgs-cpp INTERFACE
  ${CMAKE_CURRENT_BINARY_DIR})

# if(PYTHONINTERP_FOUND)
#   execute_process(
#       COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
#         from distutils import sysconfig as sc
#         print(sc.get_python_lib())"
#       OUTPUT_VARIABLE PY_INSTALL_DIR
#       OUTPUT_STRIP_TRAILING_WHITESPACE)
#   lcm_install_python(DESTINATION ${PY_INSTALL_DIR} ${python_install_sources})
# endif()
