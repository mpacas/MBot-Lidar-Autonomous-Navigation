cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(autonomous_navigation)

option(OMNIBOT "Build code for the Omnibot." OFF)

if(OMNIBOT)
  message("Building code for the Omnibot.")
  set(MACHINE_TYPE "OMNI")
  add_subdirectory(omnibot_msgs)
else()
  message("Building code for laptop.")
  set(MACHINE_TYPE "LAPTOP")
endif()

add_subdirectory(autonomous_navigation)
