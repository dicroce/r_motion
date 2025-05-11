if(NOT DEFINED OPENCV_LIBS)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        add_library(opencv INTERFACE IMPORTED GLOBAL)

        target_include_directories(opencv INTERFACE
            "$ENV{OPENCV_TOP_DIR}/include"
        )

        target_link_directories(opencv INTERFACE
            "$ENV{OPENCV_TOP_DIR}/x64/vc17/lib"
        )

        target_link_libraries(opencv INTERFACE
            opencv_core4120.lib
            opencv_imgproc4120.lib
        )

    elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        find_package(PkgConfig REQUIRED)

        pkg_search_module(OPENCV_CORE REQUIRED opencv4 opencv-core)
        pkg_search_module(OPENCV_IMGPROC REQUIRED opencv4 opencv-imgproc)

        add_library(opencv INTERFACE IMPORTED GLOBAL)

        target_include_directories(opencv INTERFACE
            ${OPENCV_CORE_INCLUDE_DIRS}
            ${OPENCV_IMGPROC_INCLUDE_DIRS}
        )

        target_link_directories(opencv INTERFACE
            ${OPENCV_CORE_LIBRARY_DIRS}
            ${OPENCV_IMGPROC_LIBRARY_DIRS}
        )

        target_link_libraries(opencv INTERFACE
            ${OPENCV_CORE_LIBRARIES}
            ${OPENCV_IMGPROC_LIBRARIES}
        )
    endif()

    set(OPENCV_LIBS opencv CACHE INTERNAL "OpenCV interface target")

endif()
