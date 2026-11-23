# ------------------------------------------------------------------------------
#                       mola_fast_lio2_wrapper
#
# Copyright (C) 2026, Jose Luis Blanco-Claraco
# SPDX-License-Identifier: BSD-3-Clause
#
# Licenses differ per target, see THIRD_PARTY_NOTICES.md:
#  - fast_lio2_core, fast_lio2_mrpt, mola-fast-lio2-cli: the program is
#    GPL-2.0-only, because of the Fast-LIO2 and ikd-Tree code. These targets
#    must never link GPL-3.0 code, MOLA libraries included.
#  - mola-fast-lio2-dataset2rawlog: GPL-3.0-only (links MOLA). A separate
#    program with no Fast-LIO2 code, connected to the former only through
#    rawlog files or pipes.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

project(mola_fast_lio2_wrapper LANGUAGES CXX)

# MOLA CMake helpers "mola_xxx()" (BSD-3-Clause, build scripts only):
find_package(mola_common REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(CLI11 REQUIRED)

# ament_cmake / BUILD_TESTING must be set up before add_subdirectory() below,
# since fast_lio2_core's own CMakeLists.txt uses ament_add_gtest():
if(DETECTED_ROS2)
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# -----------------------------------------------------------------------------
#  fast_lio2_core: the ROS-free, MOLA-free Fast-LIO2 algorithm (GPL-2.0-only)
# -----------------------------------------------------------------------------
add_subdirectory(third_party/fast_lio2_core)

# -----------------------------------------------------------------------------
#  fast_lio2_mrpt: runs fast_lio2_core on MRPT observations (not installed)
# -----------------------------------------------------------------------------
add_library(fast_lio2_mrpt STATIC
  fast_lio2_mrpt/src/Adapters.cpp
  fast_lio2_mrpt/src/Odometry.cpp
  fast_lio2_mrpt/src/yaml_env.cpp
)
mola_set_target_build_options(fast_lio2_mrpt)
target_include_directories(fast_lio2_mrpt
  PUBLIC fast_lio2_mrpt/include
  PRIVATE fast_lio2_mrpt/src
)
target_link_libraries(fast_lio2_mrpt
  PUBLIC
    mrpt::mrpt_maps
    mrpt::mrpt_obs
  PRIVATE
    fast_lio2_core
)

# -----------------------------------------------------------------------------
#  mola-fast-lio2-cli: the offline, lossless CLI app (GPL-2.0-only program)
# -----------------------------------------------------------------------------
mola_add_executable(
  TARGET  mola-fast-lio2-cli
  SOURCES apps/mola-fast-lio2-cli.cpp
  LINK_LIBRARIES
    fast_lio2_mrpt
    CLI11::CLI11
)

# Pipeline YAMLs:
install(
  DIRECTORY pipelines
  DESTINATION share/${PROJECT_NAME}
)

if(DETECTED_ROS2 AND BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(test_frame_conventions fast_lio2_mrpt/test/test_frame_conventions.cpp)
  target_link_libraries(test_frame_conventions fast_lio2_mrpt)

  ament_add_gtest(test_yaml_env fast_lio2_mrpt/test/test_yaml_env.cpp)
  target_link_libraries(test_yaml_env fast_lio2_mrpt)
endif()

# -----------------------------------------------------------------------------
#  mola-fast-lio2-dataset2rawlog: MOLA datasets to rawlog (GPL-3.0-only),
#  only built if MOLA is found.
# -----------------------------------------------------------------------------
find_package(mola_kernel QUIET)
find_package(mola_yaml QUIET)

if(mola_kernel_FOUND AND mola_yaml_FOUND)
  mola_add_executable(
    TARGET  mola-fast-lio2-dataset2rawlog
    SOURCES apps/mola-fast-lio2-dataset2rawlog.cpp
    LINK_LIBRARIES
      mola::mola_kernel
      mola::mola_yaml
      mrpt::mrpt_obs
      CLI11::CLI11
  )

  # Each dataset input is optional:
  foreach(input mulran_dataset kitti_dataset rosbag2 rosbag1)
    find_package(mola_input_${input} QUIET)
    string(TOUPPER ${input} INPUT_UPPER)
    string(REPLACE "_DATASET" "" INPUT_UPPER ${INPUT_UPPER})
    if(mola_input_${input}_FOUND)
      message(STATUS "mola-fast-lio2-dataset2rawlog: with mola_input_${input}")
      target_link_libraries(mola-fast-lio2-dataset2rawlog mola::mola_input_${input})
      target_compile_definitions(mola-fast-lio2-dataset2rawlog
        PRIVATE HAVE_MOLA_INPUT_${INPUT_UPPER})
    endif()
  endforeach()
else()
  message(STATUS "MOLA not found: mola-fast-lio2-dataset2rawlog will not be built")
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  ament_package()
endif()
