
SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/corelib/include
    ${PROJECT_SOURCE_DIR}/utilite/include
    ${PROJECT_SOURCE_DIR}/guilib/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
    ${OpenCV_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${QT_LIBRARIES} 
)

SET(SRC_FILES
    main.cpp 
)

add_definitions(${PCL_DEFINITIONS})

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

IF(MINGW)
  ADD_EXECUTABLE(odometryViewer WIN32 ${SRC_FILES})
ELSE()
  ADD_EXECUTABLE(odometryViewer ${SRC_FILES})
ENDIF()
TARGET_LINK_LIBRARIES(odometryViewer rtabmap_core rtabmap_gui rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( odometryViewer 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-odometryViewer)
  
INSTALL(TARGETS odometryViewer
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)

