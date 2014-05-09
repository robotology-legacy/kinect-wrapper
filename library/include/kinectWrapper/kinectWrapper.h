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

/** 
 * \defgroup depthSensing 3D Sensing 
 * Classes for handling 3D sensing. 
 *  
 * \defgroup kinectWrapper kinectWrapper
 * @ingroup depthSensing
 *
 * Abstract class to provide a client/server architecture in order to read
 * information from a Kinect device.
 *
 * \author Ilaria Gori
 *
 */

#ifndef __KINECTWRAPPER_H__
#define __KINECTWRAPPER_H__

#include <string>
#include <deque>
#include <map>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <kinectWrapper/kinectTags.h>

namespace kinectWrapper
{

/** 
* @ingroup kinectWrapper 
*  
* Structure to model a joint position in 2D (u,v) and in 3D 
* (x,y,z). 
*/
struct Joint
{
    int u;
    int v;
    double x;
    double y;
    double z;
};

typedef std::map<std::string, Joint> Skeleton;

/** 
* @ingroup kinectWrapper 
*  
* Structure to model a player.
*/
struct Player
{
    int ID;
    Skeleton skeleton;
};

/**
* @ingroup kinectWrapper
*
* The Definition of the KinectWrapper Interface.
*/
class KinectWrapper
{
public:
    /**
    * Configure the client or the server part of the kinectWrapper.
    * @param options contains the set of options in form of a
    *                Property object.
    *
    * Available options for the client are:
    *
    * \b remote <string>: example (remote kinectServer), specifies
    *    the server port stem-name to connect to.
    *
    * \b local <string>: example (local kinectClient), specifies the
    *    the client stem-name to be used for opening ports.
    *
    * \b carrier <string>: example (carrier udp), specifies the
    *    protocol used to connect yarp streaming ports.
    *
    * \b verbosity <int>: example (verbosity 3), specifies the
    *    verbosity level of print-outs messages.
    *
    * Available options for the server are:
    *
    * \b name <string>: example (name kinectServer), specifies the
    *    name of the server to be launched.
    *
    * \b period <int>: example (period 20), specifies the period
    *    given in [ms] for the streaming over yarp ports of data
    *    retrieved from the kinect device.
    *
    * \b verbosity <int>: example (verbosity 3), specifies the
    *    verbosity level of print-outs messages.
    *
    * \b info <string>: example (info KINECT_TAGS_ALL_INFO),
    *    specifies the info that is wanted to retrieve. If the
    *    parameter is equal to KINECT_TAGS_ALL_INFO, depth image,
    *    rgb image and skeleton information are retrieved. The
    *    alternatives are KINECT_TAGS_DEPTH,
    *    KINECT_TAGS_DEPTH_PLAYERS, KINECT_TAGS_DEPTH_RGB,
    *    KINECT_TAGS_DEPTH_RGB_PLAYERS and KINECT_TAGS_DEPTH_JOINTS.
    *
    * \b seatedMode <bool>: if the property contains this value, seatedMode will be
    *    true, otherwise will be false. It can be true only if Microsoft SDK is used.
    *    If it is true, only upper body joints are tracked.
    *
    * \b image_width <int>: example (image_width 320), specifies the
    *    width of the rgb image to send.
    *
    * \b image_height <int>: example (image_height 240), specifies the
    *    height of the rgb image to send.
    *
    * @return true/false if successful/failed.
    */
    virtual bool open(const yarp::os::Property &options) = 0;

    /**
    * Tells if the wrapper is open or not.
    * @return true/false if the wrapper is open/close.
    */
    virtual bool isOpen() = 0;

    /**
    * Retrieve the depth image.
    * @param depthIm the retrieved depth image, each pixel containing the distance in mm from
    * the sensor. If you want to draw the depth image you need to normalize the values.
    * @param timestamp when the depth image has been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getDepth(yarp::sig::ImageOf<yarp::sig::PixelMono16> &depthIm, double *timestamp) = 0;

    /**
    * Retrieve the depth image and a matrix containing information on players.
    * @param depthIm the retrieved depth image, each pixel containing the distance in meters from
    * the sensor. If you want to draw the depth image you need to normalize the values.
    * @param players a matrix containing, for each cell, which player is present.
    * @param timestamp when the depth image and the player matrix have been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getDepthAndPlayers(yarp::sig::ImageOf<yarp::sig::PixelMono16> &depthIm, yarp::sig::Matrix &players, double *timestamp) = 0;

    /**
    * Retrieve the depth image.
    * @param depthIm the retrieved depth image in float format.
    * @param timestamp when the depth image has been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getDepth(yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthIm, double *timestamp) = 0;

    /**
    * Retrieve the depth image and a matrix containing information on players.
    * @param depthIm the retrieved depth image in float format.
    * @param players a matrix containing, for each cell, which player is present.
    * @param timestamp when the depth image and the player matrix have been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getDepthAndPlayers(yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthIm, yarp::sig::Matrix &players, double *timestamp) = 0;

    /**
    * Retrieve a matrix containing information on players.
    * @param players a matrix containing, for each cell, which player is present.
    * @param timestamp when the player matrix has been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getPlayers(yarp::sig::Matrix &players, double *timestamp) = 0;

    /**
    * Retrieve the rgb image.
    * @param rgbIm the rgb image that has been retrieved.
    * @param timestamp when the rgb image has been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getRgb(yarp::sig::ImageOf<yarp::sig::PixelRgb> &rgbIm, double *timestamp) = 0;

    /**
    * Retrieve the joints position of all the players.
    * @param joints a deque of Players, each Player containing information about its joints position.
    * @param timestamp when the skeleton has been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getJoints(std::deque<Player> &joints, double *timestamp) = 0;

    /**
    * Retrieve the joints position of one player.
    * @param joints one Player containing information about its joints position.
    * @param player player ID, or KINECT_TAGS_CLOSEST_PLAYER if the closest player is wanted.
    * @param timestamp when the skeleton has been retrieved.
    * @return true/false if successful/failed.
    */
    virtual bool getJoints(Player &joints, int player, double *timestamp) = 0;

    /**
    * Retrieve some info regarding which information is retrieved from kinect,
    * whether the driver is opened in seated mode, the width and the height of
    * the rgb image.
    * @param opt Property containing all the information.
    * @return true/false if successful/failed.
    */
    virtual bool getInfo(yarp::os::Property &opt) = 0;

    /**
    * Stop the client/server and dispose it. Called by destructor.
    */
    virtual void close() = 0;

    /**
    * Fill an image with the players that are being tracked, each player with a
    * different color.
    * @param players Matrix containing, for each pixel, which player it belongs to.
    * @param image the image to be filled.
    */
    virtual void getPlayersImage(const yarp::sig::Matrix &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) = 0;

    /**
    * Fill an image with the players skeletons.
    * @param players structure containing information on the players joint positions.
    * @param image the image to be filled.
    */
    virtual void getSkeletonImage(const std::deque<Player> &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) = 0;

    /**
    * Fill an image with the player skeleton.
    * @param player structure containing information on the player joint positions.
    * @param image the image to be filled.
    */
    virtual void getSkeletonImage(const Player &player, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image) = 0;

    /**
    * Fill an image with the depth that can be displayed.
    * @param depth depth image in [mm].
    * @param depthToDisplay depth image to display.
    */
    virtual void getDepthImage(const yarp::sig::ImageOf<yarp::sig::PixelMono16> &depth, yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthToDisplay) = 0;

    /**
    * Project a pixel in 3D
    * @param u, the x coordinate of the pixel.
    * @param v, the y coordinate of the pixel.
    * @param point3D, the resultant 3D point in meters.
    * @return true/false if successful/failed.
    */
    virtual bool get3DPoint(int u, int v, yarp::sig::Vector &point3D) = 0;

    /**
     * Destructor.
     */
    virtual ~KinectWrapper() { }
};
}

#endif


