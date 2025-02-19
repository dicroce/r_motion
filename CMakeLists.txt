cmake_minimum_required(VERSION 3.14)
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)
project(r_motion VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

include(r_build.txt)
include(FetchContent)

FetchContent_Declare(
    r_utils
    GIT_REPOSITORY https://github.com/dicroce/r_utils.git
    GIT_TAG        main
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
    include
)

target_link_libraries(
    r_motion PUBLIC
    r_utils
    ${CMAKE_DL_LIBS}
)

add_subdirectory(ut)
