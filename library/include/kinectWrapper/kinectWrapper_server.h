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

#ifndef __KINECTWRAPPER_SERVER_H__
#define __KINECTWRAPPER_SERVER_H__

#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RateThread.h>
#include <kinectWrapper/kinectWrapper.h>

#ifdef __USE_SDK__
    #include <kinectWrapper/kinectDriverSDK.h>
#endif

#ifdef __USE_OPENNI__
    #include <kinectWrapper/kinectDriverOpenNI.h>
#endif

namespace kinectWrapper
{
class KinectWrapperServer : public KinectWrapper,
                  public yarp::os::RateThread,
                  public yarp::os::PortReader
{
protected:
    unsigned short* buf;
    unsigned short* bufPl;
    float* bufF;
    float* bufFPl;
    bool opening;
    bool seatedMode;
    bool useSDK;
    int period;
    int verbosity;
    int img_width;
    int img_height;
    yarp::os::Stamp tsD,tsI,tsS;
    double timestampD,timestampI,timestampS;
    std::string name;
    std::string info;

    yarp::sig::ImageOf<yarp::sig::PixelMono16> depth;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > depthPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;
    yarp::os::BufferedPort<yarp::os::Bottle> jointsPort;
    yarp::os::Bottle skeleton;

    yarp::os::Semaphore mutexDepth;
    yarp::os::Semaphore mutexRgb;
    yarp::os::Semaphore mutexSkeleton;

    yarp::os::Port rpc;

    IplImage* depthCV;
    IplImage* depthCVPl;
    IplImage* depthFCV;
    IplImage* depthFCVPl;
    IplImage* playersImage;
    IplImage* skeletonImage;
    IplImage* depthTmp;
    IplImage* depthToShow;

#ifdef __USE_SDK__
    KinectDriverSDK* driver;
#endif

#ifdef __USE_OPENNI__
    KinectDriverOpenNI* driver;
#endif

    int   printMessage(const int level, const char *format, ...) const;
    bool  read(yarp::os::ConnectionReader &connection);
    void  run();
    std::deque<Player> getJoints();
    Player getJoints(int playerId);
    Player managePlayerRequest(int playerId);
    void drawLimb(Skeleton &jointsMap, const std::string &point1, const std::string &point2);
    void threadRelease();

public:
    KinectWrapperServer();
    bool open(const yarp::os::Property &options);
    bool isOpen();
    void close();
    bool getDepth(yarp::sig::ImageOf<yarp::sig::PixelMono16> &depthIm, double *timestamp=NULL);
    bool getDepth(yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthIm, double *timestamp=NULL);
    bool getDepthAndPlayers(yarp::sig::ImageOf<yarp::sig::PixelMono16> &depthIm, yarp::sig::Matrix &players, double *timestamp=NULL);
    bool getDepthAndPlayers(yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthIm, yarp::sig::Matrix &players, double *timestamp=NULL);
    bool getPlayers(yarp::sig::Matrix &players, double *timestamp=NULL);
    bool getRgb(yarp::sig::ImageOf<yarp::sig::PixelRgb> &rgbIm, double *timestamp=NULL);
    bool getJoints(std::deque<Player> &joints, double *timestamp=NULL);
    bool getJoints(Player &joints, int player, double *timestamp=NULL);
    void getPlayersImage(const yarp::sig::Matrix &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image);
    void getSkeletonImage(const std::deque<Player> &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image);
    void getSkeletonImage(const Player &player, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image);
    void getDepthImage(const yarp::sig::ImageOf<yarp::sig::PixelMono16> &depth, yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthToDisplay);
    bool getInfo(yarp::os::Property &opt);
    bool get3DPoint(int u, int v, yarp::sig::Vector &point3D);
    virtual ~KinectWrapperServer();
};
}

#endif


