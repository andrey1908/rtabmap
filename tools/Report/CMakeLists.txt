cmake_minimum_required(VERSION 2.8)

# inside rtabmap project (see below for external build)
SET(RTABMap_INCLUDE_DIRS 
    ${PROJECT_SOURCE_DIR}/utilite/include
    ${PROJECT_SOURCE_DIR}/corelib/include
    ${PROJECT_SOURCE_DIR}/guilib/include
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

IF(QT4_FOUND OR Qt5_FOUND)
    IF(QT4_FOUND)
        INCLUDE(${QT_USE_FILE})
    ENDIF(QT4_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        ${QT_LIBRARIES}
        rtabmap_gui
    )
    ADD_DEFINITIONS("-DWITH_QT")
ENDIF(QT4_FOUND OR Qt5_FOUND)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(report main.cpp)
  
TARGET_LINK_LIBRARIES(report ${LIBRARIES})

SET_TARGET_PROPERTIES( report 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-report)
    
INSTALL(TARGETS report
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
    BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
