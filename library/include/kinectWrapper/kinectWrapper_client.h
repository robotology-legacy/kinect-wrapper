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

#ifndef __KINECTWRAPPER_CLIENT_H__
#define __KINECTWRAPPER_CLIENT_H__

#include <string>
#include <deque>

#include <cv.h>
#include <highgui.h>
#include <cxcore.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <kinectWrapper/kinectTags.h>
#include <kinectWrapper/kinectWrapper.h>

namespace kinectWrapper
{

class KinectWrapperClient : public KinectWrapper
{
protected:
    unsigned short buf[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    unsigned short bufPl[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    float bufF[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    float bufFPl[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    bool opening;
    bool init;
    bool seatedMode;
    bool drawAll;
    int verbosity;
    int img_width;
    int img_height;

    std::string remote;
    std::string local;
    std::string carrier;
    std::string info;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > depthPort;
    yarp::os::BufferedPort<yarp::os::Bottle> jointsPort;
    yarp::os::Port rpc;

    IplImage* depthCV;
    IplImage* depthCVPl;
    IplImage* depthFCV;
    IplImage* depthFCVPl;
    IplImage* playersImage;
    IplImage* skeletonImage;
    IplImage* depthTmp;
    IplImage* depthToShow;

    int printMessage(const int level, const char *format, ...) const;
    std::deque<Player> getJoints(yarp::os::Bottle *skeleton);
    Player getJoints(yarp::os::Bottle *skeleton, int playerId);
    Player managePlayerRequest(yarp::os::Bottle *skeleton, int playerId);
    void drawLimb(Skeleton &jointsMap, const std::string &point1, const std::string &point2);

public:
    KinectWrapperClient();
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
    virtual ~KinectWrapperClient();
};

}

#endif


