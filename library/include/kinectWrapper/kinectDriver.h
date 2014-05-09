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
 * \defgroup kinectDriver kinectDriver
 * @ingroup depthSensing
 *
 * Abstract class for dealing with the Kinect device. Two implementations are provided,
 * one for Microsoft SDK and the other one for OpenNI.
 *
 * \author Ilaria Gori
 *
 */

#ifndef __KINECT_DRIVER_H__
#define __KINECT_DRIVER_H__

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <kinectWrapper/kinectTags.h>

namespace kinectWrapper
{
/**
* @ingroup kinectDriver
*
* The Definition of the kinectDriver Interface.
*/
class KinectDriver
{
public:
    /**
    * Configure the driver.
    * @param opt contains the set of options in form of a
    *                Property object.
    *
    * Available options are:
    *
    * \b info <string>: example (info KINECT_TAGS_ALL_INFO), specifies
    *    the info that is wanted to retrieve. If the parameter is equal
    *    to KINECT_TAGS_ALL_INFO, depth image, rgb image and skeleton
    *    information are retrieved. The alternatives are KINECT_TAGS_DEPTH,
    *    KINECT_TAGS_DEPTH_PLAYERS, KINECT_TAGS_DEPTH_RGB, KINECT_TAGS_DEPTH_RGB_PLAYERS
    *    and KINECT_TAGS_DEPTH_JOINTS.
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
    virtual bool initialize(yarp::os::Property &opt) = 0;

    /**
    * Read the depth image from the Kinect device.
    * @param depth the read depth image.
    * @param timestamp when the depth image has been read.
    * @return true/false if successful/failed.
    */
    virtual bool readDepth(yarp::sig::ImageOf<yarp::sig::PixelMono16>& depth, double &timestamp) = 0;

    /**
    * Read the rgb image from the Kinect device.
    * @param rgb the read rgb image.
    * @param timestamp when the rgb image has been read.
    * @return true/false if successful/failed.
    */
    virtual bool readRgb(yarp::sig::ImageOf<yarp::sig::PixelRgb>& rgb, double &timestamp) = 0;

    /**
    * Read the skeleton information from the Kinect device.
    * @param skeleton a Bottle where the position of the joints is saved.
    * @param timestamp when the skeleton has been read.
    * @return true/false if successful/failed.
    */
    virtual bool readSkeleton(yarp::os::Bottle *skeleton, double &timestamp) = 0;

    /**
    * Close the driver and the device.
    * @return true/false if successful/failed.
    */
    virtual bool close() = 0;

    /**
    * Project a pixel in 3D
    * @param u, the x coordinate of the pixel.
    * @param v, the y coordinate of the pixel.
    * @param point3D, the resultant 3D point.
    * @return true/false if successful/failed.
    */
    virtual bool get3DPoint(int u, int v, yarp::sig::Vector &point3D) = 0;

    /**
    * Update all the required information.
    */
    virtual void update() = 0;
};
}

#endif


