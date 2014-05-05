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

#ifndef __KINECT_TAGS_H__
#define __KINECT_TAGS_H__

#define KINECT_TAGS_DEPTH_WIDTH             320
#define KINECT_TAGS_DEPTH_HEIGHT            240

#define KINECT_TAGS_N_JOINTS                20
#define KINECT_TAGS_MAX_USERS               15
#define KINECT_TAGS_DEVICE_KINECT           0
#define KINECT_TAGS_DEVICE_XTION            1

#define KINECT_TAGS_ALL_INFO                "all_info"
#define KINECT_TAGS_DEPTH                   "depth"
#define KINECT_TAGS_DEPTH_PLAYERS           "depth_players"
#define KINECT_TAGS_DEPTH_RGB               "depth_rgb"
#define KINECT_TAGS_DEPTH_RGB_PLAYERS       "depth_rgb_players"
#define KINECT_TAGS_DEPTH_JOINTS            "depth_joints"
#define KINECT_TAGS_CMD_PING                "ping"
#define KINECT_TAGS_CMD_ACK                 "ack"
#define KINECT_TAGS_CMD_NACK                "nack"
#define KINECT_TAGS_CMD_GET3DPOINT          "get3D"
#define KINECT_TAGS_SEATED_MODE             "seated"
#define KINECT_TAGS_CLOSEST_PLAYER          -1

#define KINECT_TAGS_BODYPART_HEAD           "head"
#define KINECT_TAGS_BODYPART_HAND_L         "handLeft"
#define KINECT_TAGS_BODYPART_HAND_R         "handRight"
#define KINECT_TAGS_BODYPART_WRIST_L        "wristLeft"
#define KINECT_TAGS_BODYPART_WRIST_R        "wristRight"
#define KINECT_TAGS_BODYPART_ELBOW_L        "elbowLeft"
#define KINECT_TAGS_BODYPART_ELBOW_R        "elbowRight"
#define KINECT_TAGS_BODYPART_SHOULDER_C     "shoulderCenter"
#define KINECT_TAGS_BODYPART_SHOULDER_L     "shoulderLeft"
#define KINECT_TAGS_BODYPART_SHOULDER_R     "shoulderRight"
#define KINECT_TAGS_BODYPART_SPINE          "spine"
#define KINECT_TAGS_BODYPART_HIP_C          "hipCenter"
#define KINECT_TAGS_BODYPART_HIP_L          "hipLeft"
#define KINECT_TAGS_BODYPART_HIP_R          "hipRight"
#define KINECT_TAGS_BODYPART_KNEE_L         "kneeLeft"
#define KINECT_TAGS_BODYPART_KNEE_R         "kneeRight"
#define KINECT_TAGS_BODYPART_ANKLE_L        "ankleLeft"
#define KINECT_TAGS_BODYPART_ANKLE_R        "ankleRight"
#define KINECT_TAGS_BODYPART_FOOT_L         "footLeft"
#define KINECT_TAGS_BODYPART_FOOT_R         "footRight"
#define KINECT_TAGS_BODYPART_COLLAR_L       "collarLeft"
#define KINECT_TAGS_BODYPART_COLLAR_R       "collarRight"
#define KINECT_TAGS_BODYPART_FT_L           "fingertipLeft"
#define KINECT_TAGS_BODYPART_FT_R           "fingertipRight"
#define KINECT_TAGS_BODYPART_COM            "CoM"

#endif


