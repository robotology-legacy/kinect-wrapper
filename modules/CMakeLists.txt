# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

if(KinectSDK_FOUND OR OpenNI_FOUND)
    add_subdirectory(kinectServer)
endif()
add_subdirectory(kinectClientExample)
