if(NOT DEFINED OPENCV_LIBS)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

        if(NOT DEFINED ENV{OPENCV_TOP_DIR})
            message(FATAL_ERROR "Environment variable OPENCV_TOP_DIR must be set on Windows.")
        endif()

        set(OPENCV_ROOT "$ENV{OPENCV_TOP_DIR}")

        add_library(opencv INTERFACE IMPORTED GLOBAL)

        target_include_directories(opencv INTERFACE
            "${OPENCV_ROOT}/include"
        )

        target_link_directories(opencv INTERFACE
            "${OPENCV_ROOT}/x64/vc17/lib"
        )

        target_link_libraries(opencv INTERFACE
            opencv_bgsegm4120.lib
            opencv_calib3d4120.lib
            opencv_core4120.lib
            opencv_features2d4120.lib
            opencv_flann4120.lib
            opencv_imgcodecs4120.lib
            opencv_imgproc4120.lib
            opencv_optflow4120.lib
            opencv_plot4120.lib
            opencv_tracking4120.lib
            opencv_video4120.lib
            opencv_xfeatures2d4120.lib
            opencv_ximgproc4120.lib
        )

    elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        find_package(PkgConfig REQUIRED)

        pkg_search_module(OPENCV_BGSEGM       REQUIRED opencv4 opencv-bgsegm)
        pkg_search_module(OPENCV_CALIB3D      REQUIRED opencv4 opencv-calib3d)
        pkg_search_module(OPENCV_CORE         REQUIRED opencv4 opencv-core)
        pkg_search_module(OPENCV_FEATURES2D   REQUIRED opencv4 opencv-features2d)
        pkg_search_module(OPENCV_FLANN        REQUIRED opencv4 opencv-flann)
        pkg_search_module(OPENCV_IMGCODECS    REQUIRED opencv4 opencv-imgcodecs)
        pkg_search_module(OPENCV_IMGPROC      REQUIRED opencv4 opencv-imgproc)
        pkg_search_module(OPENCV_OPTFLOW      REQUIRED opencv4 opencv-optflow)
        pkg_search_module(OPENCV_PLOT         REQUIRED opencv4 opencv-plot)
        pkg_search_module(OPENCV_TRACKING     REQUIRED opencv4 opencv-tracking)
        pkg_search_module(OPENCV_VIDEO        REQUIRED opencv4 opencv-video)
        pkg_search_module(OPENCV_XFEATURES2D  REQUIRED opencv4 opencv-xfeatures2d)
        pkg_search_module(OPENCV_XIMGPROC     REQUIRED opencv4 opencv-ximgproc)

        add_library(opencv INTERFACE IMPORTED GLOBAL)

        target_include_directories(opencv INTERFACE
            ${OPENCV_BGSEGM_INCLUDE_DIRS}
            ${OPENCV_CALIB3D_INCLUDE_DIRS}
            ${OPENCV_CORE_INCLUDE_DIRS}
            ${OPENCV_FEATURES2D_INCLUDE_DIRS}
            ${OPENCV_FLANN_INCLUDE_DIRS}
            ${OPENCV_IMGCODECS_INCLUDE_DIRS}
            ${OPENCV_IMGPROC_INCLUDE_DIRS}
            ${OPENCV_OPTFLOW_INCLUDE_DIRS}
            ${OPENCV_PLOT_INCLUDE_DIRS}
            ${OPENCV_TRACKING_INCLUDE_DIRS}
            ${OPENCV_VIDEO_INCLUDE_DIRS}
            ${OPENCV_XFEATURES2D_INCLUDE_DIRS}
            ${OPENCV_XIMGPROC_INCLUDE_DIRS}
        )

        target_link_directories(opencv INTERFACE
            ${OPENCV_BGSEGM_LIBRARY_DIRS}
            ${OPENCV_CALIB3D_LIBRARY_DIRS}
            ${OPENCV_CORE_LIBRARY_DIRS}
            ${OPENCV_FEATURES2D_LIBRARY_DIRS}
            ${OPENCV_FLANN_LIBRARY_DIRS}
            ${OPENCV_IMGCODECS_LIBRARY_DIRS}
            ${OPENCV_IMGPROC_LIBRARY_DIRS}
            ${OPENCV_OPTFLOW_LIBRARY_DIRS}
            ${OPENCV_PLOT_LIBRARY_DIRS}
            ${OPENCV_TRACKING_LIBRARY_DIRS}
            ${OPENCV_VIDEO_LIBRARY_DIRS}
            ${OPENCV_XFEATURES2D_LIBRARY_DIRS}
            ${OPENCV_XIMGPROC_LIBRARY_DIRS}
        )

        target_link_libraries(opencv INTERFACE
            ${OPENCV_BGSEGM_LIBRARIES}
            ${OPENCV_CALIB3D_LIBRARIES}
            ${OPENCV_CORE_LIBRARIES}
            ${OPENCV_FEATURES2D_LIBRARIES}
            ${OPENCV_FLANN_LIBRARIES}
            ${OPENCV_IMGCODECS_LIBRARIES}
            ${OPENCV_IMGPROC_LIBRARIES}
            ${OPENCV_OPTFLOW_LIBRARIES}
            ${OPENCV_PLOT_LIBRARIES}
            ${OPENCV_TRACKING_LIBRARIES}
            ${OPENCV_VIDEO_LIBRARIES}
            ${OPENCV_XFEATURES2D_LIBRARIES}
            ${OPENCV_XIMGPROC_LIBRARIES}
        )
    endif()

    set(OPENCV_LIBS opencv CACHE INTERNAL "OpenCV interface target")

endif()
