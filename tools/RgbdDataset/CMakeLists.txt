cmake_minimum_required(VERSION 2.8)

# inside rtabmap project (see below for external build)
SET(RTABMap_INCLUDE_DIRS 
    ${PROJECT_SOURCE_DIR}/utilite/include
    ${PROJECT_SOURCE_DIR}/corelib/include
)
SET(RTABMap_LIBRARIES 
    rtabmap_core
    rtabmap_utilite
)  

if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
endif()

SET(INCLUDE_DIRS
    ${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${RTABMap_LIBRARIES}
    ${OpenCV_LIBRARIES}
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

ADD_EXECUTABLE(rgbd_dataset main.cpp)
  
TARGET_LINK_LIBRARIES(rgbd_dataset ${LIBRARIES})


SET_TARGET_PROPERTIES( rgbd_dataset 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-rgbd_dataset)
    
INSTALL(TARGETS rgbd_dataset
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
    BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
