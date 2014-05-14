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

#ifndef __KINECT_DRIVER_SDK_H__
#define __KINECT_DRIVER_SDK_H__

#include <windows.h>
#include <ole2.h>
#include "NuiApi.h"
#include "NuiImageCamera.h"
#include "NuiSensor.h"
#include "NuiSkeleton.h"
#include <yarp/os/Semaphore.h>
#include <kinectWrapper/kinectDriver.h>

namespace kinectWrapper
{
class KinectDriverSDK: public KinectDriver
{
    IplImage* color;
    IplImage* depthTmp;

    USHORT* buf;
    HANDLE h1,h2,h3,h4;

    bool seatedMode;
    bool initC;
    bool initD;
    std::string info;
    int img_height;
    int img_width;
    int def_image_height;
    int def_image_width;

private:
    int setColorImg(HANDLE h,IplImage* color,const NUI_IMAGE_FRAME * pImageFrame);
    int setDepthImg(HANDLE h,IplImage* depth, const NUI_IMAGE_FRAME * pImageFrame );
    const NUI_IMAGE_FRAME * retrieveImg(HANDLE h);
    std::string jointNameAssociation(int id);
    bool isJointProvided(int id);

public:

    KinectDriverSDK()
    {
    }
    bool initialize(yarp::os::Property &opt);
    bool readRgb(yarp::sig::ImageOf<yarp::sig::PixelRgb> &rgb, double &timestamp);
    bool readDepth(yarp::sig::ImageOf<yarp::sig::PixelMono16> &depth, double &timestamp);
    bool readSkeleton(yarp::os::Bottle *skeleton, double &timestamp);
    bool get3DPoint(int u, int v, yarp::sig::Vector &point3D);
    bool close();
    void update();
};
}

#endif


