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

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(kitti_dataset main.cpp)
  
TARGET_LINK_LIBRARIES(kitti_dataset ${LIBRARIES})

SET_TARGET_PROPERTIES( kitti_dataset 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-kitti_dataset)
    
INSTALL(TARGETS kitti_dataset
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
    BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
