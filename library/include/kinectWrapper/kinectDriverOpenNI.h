/* Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori
 * email:   ilaria.gori@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found in the file LICENSE located in the
 * root directory.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __KINECT_DRIVER_OPENNI_H__
#define __KINECT_DRIVER_OPENNI_H__

#include <stdio.h>
#include <stdarg.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <yarp/os/Network.h>

#include <XnCppWrapper.h>

#include <kinectWrapper/kinectTags.h>
#include <kinectWrapper/kinectDriver.h>

namespace kinectWrapper
{
class KinectDriverOpenNI: public KinectDriver
{
private:
    bool seatedMode;
    bool requireCalibrationPose;
    bool requireRemapping;
    std::string info;
    int img_height;
    int img_width;
    int def_image_height;
    int def_image_width;
    int def_depth_width;
    int def_depth_height;
    int device;

    IplImage* rgb_big;
    IplImage* depthTmp;
    IplImage* depthImage;
    CvMat* depthMat;

    xn::Context context;
    xn::DepthGenerator depthGenerator;
    xn::ImageGenerator imageGenerator;
    xn::UserGenerator userGenerator;

    bool testRetVal(XnStatus nRetVal, std::string message);
    void resizeImage(IplImage* depthTmp, IplImage* depthImage);
    std::string jointNameAssociation(XnSkeletonJoint joint);

public:
    bool initialize(yarp::os::Property &opt);
    bool readDepth(yarp::sig::ImageOf<yarp::sig::PixelMono16> &depth, double &timestamp);
    bool readRgb(yarp::sig::ImageOf<yarp::sig::PixelRgb> &rgb, double &timestamp);
    bool readSkeleton(yarp::os::Bottle *skeleton, double &timestamp);
    bool get3DPoint(int u, int v, yarp::sig::Vector &point3D);
    bool close();
    void update();
    bool getRequireCalibrationPose();
};
}

#endif


