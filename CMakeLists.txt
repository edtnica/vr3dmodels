cmake_minimum_required(VERSION 3.16.3)
project(vr3dmodels VERSION 0.1.0 LANGUAGES C CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(OpenCV_DIR "/home/csimage/OpenCV/build")
find_package(OpenCV REQUIRED)

# link_directories(${OpenCV_LIB_DIR})

find_package(Ceres REQUIRED)

find_package(PCL 1.8 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(
    vr3dmodels
    ./src/main.cpp
    ./src/sfm.cpp
    ./src/sfmBundleAdjustment.cpp
    ./src/cameraCalibration.cpp)

target_link_libraries(vr3dmodels
    ${OpenCV_LIBS}
    Ceres::ceres
    ${PCL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
