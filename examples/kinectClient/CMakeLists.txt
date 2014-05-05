# Copyright: (C) 2011 EFAA Consortium
# Authors: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME kinectClient)
project(${PROJECTNAME})

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${kinect_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} kinect ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

