cmake_minimum_required(VERSION 3.10)
project(feature_detection)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV package with required components
find_package(OpenCV REQUIRED COMPONENTS 
    xfeatures2d 
    aruco
    videoio 
    highgui 
    calib3d
)

# Add the camera calibration executable
add_executable(camera_calibration 
    camera_calibration.cpp
    calibration.cpp
    calibration.hpp
)

# Add the detector executable
add_executable(detector
    detectconer.cpp
)

# Add camera executable
add_executable(camera camera.cpp)

# Add camera position estimation executable
add_executable(position position.cpp)

# Add 3D axis visualization executable
add_executable(3daxe 3daxe.cpp)

# Add virtual object visualization executable
add_executable(3dobject 3dobject.cpp)

# Add features
add_executable(feature feature.cpp)


# Link OpenCV libraries to all executables
target_link_libraries(camera_calibration ${OpenCV_LIBS})
target_link_libraries(detector ${OpenCV_LIBS})
target_link_libraries(camera ${OpenCV_LIBS})
target_link_libraries(position ${OpenCV_LIBS})
target_link_libraries(3daxe ${OpenCV_LIBS})
target_link_libraries(3dobject ${OpenCV_LIBS})
target_link_libraries(feature ${OpenCV_LIBS})


# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
