cmake_minimum_required(VERSION 3.6)

SET(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)
SET(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE PATH "Path to your vcpkg")

project(face_reconstruction)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
# set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
# set(dlib_DIR ${LIBRARY_DIR}/DLib/lib/cmake/dlib CACHE PATH "Path to installed DLib")
# set(OpenCV_DIR  ${LIBRARY_DIR}/OpenCV CACHE PATH "Path to installed OpenCV")
# set(PCL_DIR  ${LIBRARY_DIR}/PCL/cmake CACHE PATH "Path to installed PCL")

# Use pre-compiled libraries on Windows
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    endif()
endif(WIN32)

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(dlib REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(HDF5 REQUIRED COMPONENTS C CXX)
FIND_PACKAGE(Ceres CONFIG REQUIRED)
FIND_PACKAGE(PCL CONFIG REQUIRED)

#FIND_PACKAGE(OpenGL REQUIRED)
#FIND_PACKAGE(GLUT REQUIRED)

set(HEADERS 
    Eigen.h
    VirtualSensor.h
    MeshFileHelper.h
    FreeImageHelper.h
    OpenCVImgCropper.h
    LandmarkDetector.h  
    BFMContainer.h  
    BFMOptimizer.h  
    PointCloud.h
    ProcrustesAligner.h
    SimpleMesh.h
    common.h
)
# Utils/json11.hpp
# Utils/json11.cpp
set(SOURCES
    OpenCVImgCropper.cpp 
    LandmarkDetector.cpp
    FreeImageHelper.cpp
    main.cpp
    FreeImageHelper.cpp
    BFMContainer.cpp  
    BFMOptimizer.cpp  
)

link_directories(${HDF5_CXX_INCLUDE_DIRS})
link_directories(${dlib_INCLUDE_DIRS})
link_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${FreeImage_LIBRARY_DIR})
link_directories(${FreeImage_LIBRARY_DIR})

add_executable(face_reconstruction ${HEADERS} ${SOURCES})

target_include_directories(face_reconstruction PUBLIC ${HDF5_CXX_INCLUDE_DIRS})
target_include_directories(face_reconstruction PUBLIC ${dlib_INCLUDE_DIRS})
target_include_directories(face_reconstruction PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(face_reconstruction PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(face_reconstruction PUBLIC ${FreeImage_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(face_reconstruction dlib::dlib)
TARGET_LINK_LIBRARIES(face_reconstruction ${HDF5_CXX_LIBRARIES})
TARGET_LINK_LIBRARIES(face_reconstruction ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(face_reconstruction Eigen3::Eigen general freeimage)
TARGET_LINK_LIBRARIES(face_reconstruction ceres Ceres::ceres)
TARGET_LINK_LIBRARIES(face_reconstruction ${PCL_LIBRARIES})

#TARGET_LINK_LIBRARIES(face_reconstruction ${OPENGL_gl_LIBRARY})
#TARGET_LINK_LIBRARIES(face_reconstruction GLUT::GLUT)


if(WIN32)
    # Visual Studio properties
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT face_reconstruction)
    set_property(TARGET face_reconstruction PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

    # On Windows copy dll to executable folder
    add_custom_command(TARGET face_reconstruction POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${FreeImage_LIBRARY_DIR}/FreeImage.dll"
            ${CMAKE_BINARY_DIR})
endif(WIN32)