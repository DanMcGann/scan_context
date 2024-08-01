# Setup gtest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        origin/main
)
option(INSTALL_GTEST OFF)
option(BUILD_GMOCK OFF)
FetchContent_MakeAvailable(googletest)
enable_testing() # enable gtest

# Make test target
file(GLOB_RECURSE tests_srcs *.cpp)
add_executable(scan-context-tests ${tests_srcs})
target_link_libraries(scan-context-tests PUBLIC ScanContext Eigen3::Eigen gtest gtest_main)
add_test(NAME scan-context-tests COMMAND scan-context-tests)

# Make runnable from "make check" or "make test"
add_custom_target(scan-context-check COMMAND scan-context-tests)
add_custom_target(scan-context-test COMMAND scan-context-tests)