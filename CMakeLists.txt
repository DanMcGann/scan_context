project(ScanContext CXX C)
cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 17)

message(STATUS "================  BUILDING SCANCONTEXT  ======================")

# Dependencies
# Eigen is a base dependency for matrix types
find_package(Eigen3 REQUIRED)

# nanoflann is a base dependency for KNN search in the database
include(FetchContent)
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG        v1.5.5
)
FetchContent_MakeAvailable(nanoflann)
set(NANOFLANN_BUILD_EXAMPLES OFF)


# Define the library
add_library(ScanContext INTERFACE)
target_link_libraries(ScanContext INTERFACE Eigen3::Eigen nanoflann)
target_include_directories(ScanContext INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")


# If testing is configured, setup the tests
if (${ENABLE_TESTS})
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tests")
endif()


# If python bindings are configured, build the python bindings
if (${ENABLE_PYTHON})
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/python")
endif()
