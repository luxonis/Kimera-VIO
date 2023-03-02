cmake_minimum_required(VERSION 2.8.2)

project(googletest-download NONE)

include(ExternalProject)
ExternalProject_Add(backward
    GIT_REPOSITORY https://github.com/bombela/backward-cpp.git
    GIT_TAG v1.6 # Last working version
    SOURCE_DIR "${CMAKE_BINARY_DIR}/external/backward-cpp-src"
    BINARY_DIR "${CMAKE_BINARY_DIR}/external/backward-cpp-build"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    TEST_COMMAND ""
)
