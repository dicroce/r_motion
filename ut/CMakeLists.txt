
FetchContent_Declare(
    r_av
    GIT_REPOSITORY https://github.com/dicroce/r_av.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_av)

add_executable(
    r_motion_ut
    include/framework.h
    source/framework.cpp
    include/test_r_motion.h
    source/test_r_motion.cpp
)

if(MSVC)
    target_link_options(r_motion_ut PUBLIC "/STACK:10000000")
endif()

target_include_directories(
    r_motion_ut PUBLIC 
    include
    ../include
)
target_link_libraries(
    r_motion_ut LINK_PUBLIC 
    r_utils
    r_av
)
