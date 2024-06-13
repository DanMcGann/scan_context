project(ScanContext CXX C)
cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_SKIP_INSTALL_RULES YES)

message(STATUS "================  BUILDING SCANCONTEXT  ======================")

# Dependencies
# Eigen is a base dependency for matrix types
find_package(Eigen3 REQUIRED)

# nanoflann is a base dependency for KNN search in the database
include(FetchContent)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "Do not build nanoflann examples")
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "Do not build nanoflann tests")
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG        v1.5.5
)
FetchContent_MakeAvailable(nanoflann)

# Define the library
file(GLOB scan_context_srcs "${CMAKE_CURRENT_SOURCE_DIR}/scan_context/src/*.cpp")
file(GLOB scan_context_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/scan_context/include/scan_context/*.h")

add_library(ScanContext SHARED ${scan_context_srcs} ${scan_context_hdrs})
target_link_libraries(ScanContext PUBLIC Eigen3::Eigen nanoflann)
target_include_directories(ScanContext PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/scan_context/include")


# If testing is configured, setup the tests
if (${SCAN_CONTEXT_BUILD_TESTS})
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tests")
endif()


# If python bindings are configured, build the python bindings
if (${SCAN_CONTEXT_BUILD_PYTHON})
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/python")
endif()
