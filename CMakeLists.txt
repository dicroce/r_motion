cmake_minimum_required(VERSION 3.28.3)
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)
project(r_motion VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

include(r_build.txt)
include(FetchContent)
include(OpenCVConfig.cmake)

FetchContent_Declare(
    r_utils
    GIT_REPOSITORY https://github.com/dicroce/r_utils.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_utils
)
FetchContent_MakeAvailable(r_utils)

add_library(
    r_motion
    include/r_motion/r_motion_state.h
    source/r_motion_state.cpp
    include/r_motion/utils.h
    source/utils.cpp
)

target_include_directories(
    r_motion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    r_motion PUBLIC
    r_utils
    ${OPENCV_LIBS}
)

option(BUILD_INSTALL_DEV_PACKAGES "Install dev packages for subprojects" OFF)

#add_subdirectory(ut)

if(PROJECT_IS_TOP_LEVEL OR BUILD_INSTALL_DEV_PACKAGES)
    configure_dev_package("r_motion" "A C++ library that does video motion detection.")
endif()
