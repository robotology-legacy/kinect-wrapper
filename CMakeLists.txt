# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.7)

set(PROJECTNAME kinectWrapper)
project(${PROJECTNAME})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/library/cmake)

option(BUILD_CLIENT_ONLY "" FALSE)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

if(BUILD_CLIENT_ONLY)
  message("Only the client will be built.")
else()  
  find_package(KinectSDK)
  find_package(OpenNI)
endif()

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

find_package(OpenCV REQUIRED)

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES})   
include(YarpInstallationHelpers) 

add_subdirectory(library)
add_subdirectory(modules)

icubcontrib_finalize_export(${PROJECTNAME})
icubcontrib_add_uninstall_target()

