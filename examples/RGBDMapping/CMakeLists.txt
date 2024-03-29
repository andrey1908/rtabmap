cmake_minimum_required(VERSION 2.8)

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(internal)
    # inside rtabmap project (see below for external build)
    SET(RTABMap_INCLUDE_DIRS 
        ${PROJECT_SOURCE_DIR}/utilite/include
        ${PROJECT_SOURCE_DIR}/corelib/include
        ${PROJECT_SOURCE_DIR}/guilib/include
    )
    SET(RTABMap_LIBRARIES 
        rtabmap_core
        rtabmap_gui
        rtabmap_utilite
    )  
else()
    # external build
    PROJECT( MyProject )
    
    FIND_PACKAGE(RTABMap REQUIRED)
    FIND_PACKAGE(OpenCV REQUIRED)
    FIND_PACKAGE(PCL 1.7 REQUIRED)
    
    # Find Qt5 first
    FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
        IF(NOT Qt5_FOUND)
               FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
        ENDIF(NOT Qt5_FOUND)

    # fix libproj.so not found on Xenial
    if(NOT "${PCL_LIBRARIES}" STREQUAL "")
           list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
    endif()

endif()

if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
endif()

SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${QT_LIBRARIES} 
    ${PCL_LIBRARIES}
)

# Hack as CameraRealsense2.h needs realsense2 include dir
IF(realsense2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIRS}
    )
ENDIF(realsense2_FOUND)

# Hack as CameraK4A.h needs k4a include dir
IF(k4a_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${k4a_INCLUDE_DIRS}
    )
ENDIF(k4a_FOUND)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs MapBuilder.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h)
ENDIF()

ADD_EXECUTABLE(rgbd_mapping main.cpp ${moc_srcs})
  
TARGET_LINK_LIBRARIES(rgbd_mapping ${LIBRARIES})

if(internal)
  SET_TARGET_PROPERTIES( rgbd_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-rgbd_mapping)
endif(internal)
