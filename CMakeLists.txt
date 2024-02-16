cmake_minimum_required(VERSION 3.16.3)
project(vr3dmodels VERSION 0.1.0 LANGUAGES C CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(OpenCV_DIR "/home/csimage/OpenCV/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# link_directories(${OpenCV_LIB_DIR})

find_package(Ceres REQUIRED)

add_executable(vr3dmodels main.cpp sfm.cpp cameraCalibration.cpp)

target_link_libraries(vr3dmodels ${OpenCV_LIBS})
target_link_libraries(vr3dmodels Ceres::ceres)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
