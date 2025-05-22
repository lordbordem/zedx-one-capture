cmake_minimum_required(VERSION 3.5)
project(argus_camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Find GStreamer packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
include_directories(${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER_BASE_INCLUDE_DIRS} ${GSTREAMER_APP_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS} ${GSTREAMER_BASE_LIBRARY_DIRS} ${GSTREAMER_APP_LIBRARY_DIRS})

set(TegraMM_ROOT /usr/src/jetson_multimedia_api)
set(TegraMM_FOUND FALSE)

if(EXISTS ${TegraMM_ROOT})
  # set packages
  set(TegraMM_INCLUDE_DIRS ${TegraMM_ROOT}/include ${TegraMM_ROOT}/include/libjpeg-8b /usr/include/libdrm)
  set(TegraMM_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/tegra /usr/lib/aarch64-linux-gnu)
  set(TegraMM_LIBRARIES nvargus_socketclient nvjpeg drm nvbufsurftransform nvbufsurface nvosd EGL v4l2 GLESv2 X11 pthread vulkan)
  file(GLOB TegraMM_COMMON_SOURCES ${TegraMM_ROOT}/samples/common/classes/*.cpp)
  include_directories(${TegraMM_INCLUDE_DIRS})
  link_directories(${TegraMM_LIBRARY_DIRS})
  set(TegraMM_FOUND TRUE)
endif()

add_definitions(-std=c++14 -O3)
IF (BUILD_WITH_DEBUGINFOS)
    message("!! Building with -g !!")
    add_definitions(-g)
    set(ARGUS_LIB arguscaptured)
ELSE()
    set(ARGUS_LIB arguscapture)
ENDIF()

add_executable(argus_camera main.cpp)
target_link_libraries(argus_camera ${ARGUS_LIB} pthread ${OpenCV_LIBRARIES} ${TegraMM_LIBRARIES} 
                      ${GSTREAMER_LIBRARIES} ${GSTREAMER_BASE_LIBRARIES} ${GSTREAMER_APP_LIBRARIES})
