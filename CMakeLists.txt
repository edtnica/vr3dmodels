cmake_minimum_required(VERSION 3.25)
project(vr3dmodels VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(OpenCV_DIR "C:/Programs and Stuff/OpenCV/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# link_directories(${OpenCV_LIB_DIR})

add_executable(vr3dmodels main.cpp sfm.cpp cameraCalibration.cpp)

target_link_libraries(vr3dmodels ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
