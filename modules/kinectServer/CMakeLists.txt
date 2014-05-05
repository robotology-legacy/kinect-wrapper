# Copyright: (C) 2011 EFAA Consortium
# Authors: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME kinectServer)
project(${PROJECTNAME})

if (EFAA_USE_KinectSDK AND EFAA_USE_OpenNI)
	if (KINECT_DRIVER_USE_SDK)
		set(KINECT_DRIVER_USE_OpenNI FALSE)
		add_definitions(-D__EFAA_KINECT_USE_SDK__)
	elseif (NOT KINECT_DRIVER_USE_SDK)
		set(KINECT_DRIVER_USE_OpenNI TRUE)
		add_definitions(-D__EFAA_KINECT_USE_OPENNI__)
	endif(KINECT_DRIVER_USE_SDK)
elseif (EFAA_USE_KinectSDK AND (NOT EFAA_USE_OpenNI))
	set(KINECT_DRIVER_USE_SDK TRUE)
	add_definitions(-D__EFAA_KINECT_USE_SDK__)
elseif ((NOT EFAA_USE_KinectSDK) AND EFAA_USE_OpenNI)
	set(KINECT_DRIVER_USE_OpenNI TRUE)
	add_definitions(-D__EFAA_KINECT_USE_OPENNI__)
endif (EFAA_USE_KinectSDK AND EFAA_USE_OpenNI)

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${kinect_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} kinect ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

