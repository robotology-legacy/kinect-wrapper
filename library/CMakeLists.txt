# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

MACRO(CMAKE_DEPENDENT_OPTION option doc default depends force)
  IF(${option}_ISSET MATCHES "^${option}_ISSET$")
    SET(${option}_AVAILABLE 1)
    FOREACH(d ${depends})
      STRING(REGEX REPLACE " +" ";" CMAKE_DEPENDENT_OPTION_DEP "${d}")
      IF(${CMAKE_DEPENDENT_OPTION_DEP})
      ELSE(${CMAKE_DEPENDENT_OPTION_DEP})
        SET(${option}_AVAILABLE 0)
      ENDIF(${CMAKE_DEPENDENT_OPTION_DEP})
    ENDFOREACH(d)
    IF(${option}_AVAILABLE)
      OPTION(${option} "${doc}" "${default}")
      SET(${option} "${${option}}" CACHE BOOL "${doc}" FORCE)
    ELSE(${option}_AVAILABLE)
      IF(${option} MATCHES "^${option}$")
      ELSE(${option} MATCHES "^${option}$")
        SET(${option} "${${option}}" CACHE INTERNAL "${doc}")
      ENDIF(${option} MATCHES "^${option}$")
      SET(${option} ${force})
    ENDIF(${option}_AVAILABLE)
  ELSE(${option}_ISSET MATCHES "^${option}_ISSET$")
    SET(${option} "${${option}_ISSET}")
  ENDIF(${option}_ISSET MATCHES "^${option}_ISSET$")
ENDMACRO(CMAKE_DEPENDENT_OPTION)

cmake_minimum_required(VERSION 2.8.7)
set(PROJECTNAME kinectWrapper)
project(${PROJECTNAME})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(KinectSDK)
find_package(OpenNI)

if (NOT KinectSDK_FOUND AND NOT OpenNI_FOUND)
   message(FATAL_ERROR "At least one between KinectSDK and OpenNI packages must be available!")
endif ()

CMAKE_DEPENDENT_OPTION(USE_KinectSDK "Use KinectSDK" ON "KinectSDK_FOUND;NOT OpenNI_FOUND" OFF)
if (USE_KinectSDK AND NOT KinectSDK_FOUND)
   message(STATUS "you want to use KinectSDK but it is not available on your system; we switch to OpenNI.")
   set(USE_KinectSDK OFF)
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}) 

set(headers include/kinectWrapper/kinectTags.h
            include/kinectWrapper/kinectWrapper.h
            include/kinectWrapper/kinectWrapper_client.h)
set(sources src/kinectWrapper_client.cpp)

if (USE_KinectSDK)
   include_directories(${KinectSDK_INCLUDE_DIRS})
   LINK_DIRECTORIES(${KinectSDK_LIB_DIR})
   add_definitions(-D__USE_SDK__)
   list(APPEND headers include/kinectWrapper/kinectDriver.h
                       include/kinectWrapper/kinectDriverSDK.h
                       include/kinectWrapper/kinectWrapper_server.h)
   list(APPEND sources src/kinectDriverSDK.cpp
                       src/kinectWrapper_server.cpp)
else ()
   include_directories(${OpenNI_INCLUDE_DIRS})
   add_definitions(-D__USE_OPENNI__)
   list(APPEND headers include/kinectWrapper/kinectDriver.h
                       include/kinectWrapper/kinectDriverOpenNI.h
                       include/kinectWrapper/kinectWrapper_server.h)
   list(APPEND sources src/kinectDriverOpenNI.cpp
                       src/kinectWrapper_server.cpp)
endif ()

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

add_library(${PROJECTNAME} ${headers} ${sources})

if (WIN32)
    set_target_properties(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
endif ()

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

if (USE_KinectSDK)
    target_link_libraries(${PROJECTNAME} ${KinectSDK_LIBRARIES})
    icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                              EXTERNAL_INCLUDE_DIRS ${YARP_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${KinectSDK_INCLUDE_DIRS}
                                              DESTINATION include/kinectWrapper
                                              DEPENDS ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${KinectSDK_LIBRARIES}
                                              FILES ${headers})
else ()
    target_link_libraries(${PROJECTNAME} ${OpenNI_LIBRARIES})
    icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                              EXTERNAL_INCLUDE_DIRS ${YARP_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${OpenNI_INCLUDE_DIR}
                                              DESTINATION include/kinectWrapper
                                              DEPENDS ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenNI_LIBRARIES}
                                              FILES ${headers})
endif ()

#icubcontrib_finalize_export(${PROJECTNAME})
#icubcontrib_add_uninstall_target()


