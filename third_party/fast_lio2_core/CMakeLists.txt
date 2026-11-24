# fast_lio2_core: the Fast-LIO2 algorithm, stripped of ROS and MOLA
# dependencies. GPL-2.0-only, see ../../THIRD_PARTY_NOTICES.md.
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)

add_library(fast_lio2_core STATIC
  src/ImuProcess.cpp
  src/FastLio2Core.cpp
  # Compiles ../ikd-Tree/ikd_Tree.cpp (upstream, unmodified):
  src/ikd_Tree_build.cpp
)

# SYSTEM: upstream-derived headers, kept close to upstream, so our stricter
# warning flags (e.g. -Wshadow) are not applied to them.
target_include_directories(fast_lio2_core SYSTEM PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../          # for <IKFoM_toolkit/...>
  ${CMAKE_CURRENT_SOURCE_DIR}/../ikd-Tree  # for <ikd_Tree.h>
)

target_link_libraries(fast_lio2_core PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::boost
  OpenMP::OpenMP_CXX
)
# See the comment at the top of src/FastLio2Core.cpp: forces PCL filter
# templates (VoxelGrid) to be instantiated from headers instead of linked
# from the system libpcl_filters.so, to avoid a real ABI mismatch observed
# between the distro PCL package and the installed Eigen headers. PUBLIC so
# consumers linking PointCloudXYZI::Ptr across this library's API boundary
# stay consistent.
target_compile_definitions(fast_lio2_core PUBLIC PCL_NO_PRECOMPILE)

# ikd-Tree hands any subtree above Multi_Thread_Rebuild_Point_Num to an
# asynchronous rebuild thread, and only if a mutex trylock succeeds, so results
# depend on thread timing and runs are not reproducible. Setting the threshold
# past any reachable subtree size forces every rebuild inline: runs become
# bit-exact, at some cost in speed and none measured in accuracy. Needed for
# perturbation and sensitivity studies; off by default. The threshold is set in
# src/ikd_Tree_build.cpp, so the upstream ikd-Tree sources stay unmodified.
option(FASTLIO2_SYNC_IKD_REBUILD
  "Rebuild the ikd-Tree inline instead of on a background thread (deterministic)"
  OFF)
if(FASTLIO2_SYNC_IKD_REBUILD)
  target_compile_definitions(fast_lio2_core PRIVATE FASTLIO2_SYNC_IKD_REBUILD)
endif()
target_include_directories(fast_lio2_core PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(fast_lio2_core PUBLIC ${PCL_DEFINITIONS})

set_target_properties(fast_lio2_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_fast_lio2_core test/test_fast_lio2_core.cpp)
    target_link_libraries(test_fast_lio2_core fast_lio2_core)
  endif()
endif()
