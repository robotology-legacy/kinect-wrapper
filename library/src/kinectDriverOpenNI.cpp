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

#include <kinectWrapper/kinectDriverOpenNI.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace kinectWrapper;
using namespace xn;

/************************************************************************/
bool KinectDriverOpenNI::initialize(Property &opt)
{
    this->info=opt.check("info",Value(KINECT_TAGS_ALL_INFO)).asString().c_str();
    this->seatedMode=false;
    this->img_width=opt.check("img_width",Value(320)).asInt();
    this->img_height=opt.check("img_height",Value(240)).asInt();
    this->requireRemapping=opt.check("remap");

    openni::Status rc=openni::OpenNI::initialize();
    
    if (!testRetVal(rc, "Inialization"))
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
        return false;
    }

    //context.SetGlobalMirror(true); TOCHECK LATER
    //VideoStream::setMirroringEnabled();

    rc=device.open(openni::ANY_DEVICE);
    if (!testRecVal(rc, "Opening device"))
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return false;       
    }

    if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);){
        device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }
    
    rc=depthStream.create(device, openni::SENSOR_DEPTH);
    if (!testRecVal(rc, "Opening depth"))
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return false;
    }
    
    openni::VideoMode depth_videoMode  = depthStream.getVideoMode();
    this->def_depth_width=depth_videoMode.getResolutionX();
    this->def_depth_height=depth_videoMode.getResolutionY();
    
    rc=depthStream.start();
    if (!testRecVal(rc, "Starting depth"))
    {
        printf("%s\n", openni::OpenNI::getExtendedError());
        depthStream.destroy();
        openni::OpenNI::shutdown();
        return false;
    }

    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        rc=imageStream.create(device, openni::SENSOR_COLOR);
        if (!testRecVal(rc, "Opening RGB"))
        {
            printf("%s\n", openni::OpenNI::getExtendedError());
            depthStream.destroy();
            openni::OpenNI::shutdown();
            return false;
        }
        
        rc=imageStream.start();
        if (!testRecVal(rc, "Starting RGB"))
        {
            printf("%s\n", openni::OpenNI::getExtendedError());
            depthStream.destroy();
            imageStream.destroy();
            openni::OpenNI::shutdown();
            return false;
        }
        
        openni::VideoMode image_videoMode  = imageStream.getVideoMode();
        this->def_image_width=image_videoMode.getResolutionX();
        this->def_image_height=image_videoMode.getResolutionY();
    }

    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS || info==KINECT_TAGS_DEPTH_PLAYERS || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        nite::Status niteRc=nite::NiTE::initialize();
        niteRc=userTracker.create(&device);
        
        if (niteRC!=nite::STATUS_OK)
        {
            printf("Cannot create user tracker\n");
            depthStream.destroy();
            if (imageStream.isValid())
                imageStream.destroy();
            openni::OpenNI::shutdown();
        }
    }
    
    depthTmp=cvCreateImage(cvSize(def_depth_width,def_depth_height),IPL_DEPTH_16U,1);
    depthImage=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    rgb_big=cvCreateImageHeader(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,3);
    depthMat = cvCreateMat(def_depth_height,def_depth_width,CV_16UC1);

    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::readDepth(ImageOf<PixelMono16> &depth, double &timestamp)
{
    int ts=(int)userFrame.getTimestamp();
    timestamp=(double)ts/1000.0;

    const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
    
    const nite::UserId* pLabels = userFrame.getUserMap().getPixels();

    for (int y=0; y<this->def_depth_height; y++)
    {
        for(int x=0; x<this->def_depth_width; x++, pLabels++, pDepth++)
        {
            int player=*pLabels;
            int depth=*pDepth;
            int finalValue=0;
            finalValue|=((depth<<3)&0XFFF8);
            finalValue|=(player&0X0007);
            //if (x==320 && y==240)
             //   fprintf(stdout, "%d %d\n", ((finalValue&0XFFF8)>>3), finalValue&0X0007);
            //We associate the depth to the first 13 bits, using the last 3 for the player identification
            depthMat->data.s[y * this->def_depth_width + x ]=finalValue;
        }
    }

    cvGetImage(depthMat,depthTmp);
    resizeImage(depthTmp,depthImage);

    depth.wrapIplImage(depthImage);

    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::readRgb(ImageOf<PixelRgb> &rgb, double &timestamp)
{
    int ts=(int)imageStream.getTimestamp();
    timestamp=(double)ts/1000.0;
    const XnRGB24Pixel* pImage = imageGenerator.GetRGB24ImageMap();
    XnRGB24Pixel* ucpImage = const_cast<XnRGB24Pixel*> (pImage);
    cvSetData(rgb_big,ucpImage,this->def_image_width*3);
    cvResize(rgb_big,(IplImage*)rgb.getIplImage());
    int ts=(int)imageGenerator.GetTimestamp();
    timestamp=(double)ts/1000.0;
    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::readSkeleton(Bottle *skeleton, double &timestamp)
{
    skeleton->clear();
    int ts=(int)userFrame.getTimestamp();
    timestamp=(double)ts/1000.0;
    bool isTracking=false;
    if((info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS))
    {
        Bottle bones;
        bones.clear();
        const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
        for (int i = 0; i < users.getSize(); ++i)
        {
            const nite::UserData& user = users[i];
            updateUserState(user,userFrame.getTimestamp());
            
            if (user.isNew())
            {
                userTracker.startSkeletonTracking(user.getId());
            }
            else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
            {
                isTracking=true;
                yarp::sig::Vector com(3); com=0.0;
                Bottle &player=bones.addList();
                player.addInt(user.getId());
                    
                updatePlayer(nite::JOINT_HEAD,player,user,com);
                updatePlayer(nite::JOINT_NECK,player,user,com);
                updatePlayer(nite::JOINT_LEFT_SHOULDER,player,user,com);
                updatePlayer(nite::JOINT_RIGHT_SHOULDER,player,user,com);
                updatePlayer(nite::JOINT_LEFT_ELBOW,player,user,com);
                updatePlayer(nite::JOINT_RIGHT_ELBOW,player,user,com);
                updatePlayer(nite::JOINT_LEFT_HAND,player,user,com);
                updatePlayer(nite::JOINT_RIGHT_HAND,player,user,com);
                updatePlayer(nite::JOINT_TORSO,player,user,com);
                updatePlayer(nite::JOINT_LEFT_HIP,player,user,com);
                updatePlayer(nite::JOINT_RIGHT_HIP,player,user,com);
                updatePlayer(nite::JOINT_LEFT_KNEE,player,user,com);
                updatePlayer(nite::JOINT_RIGHT_KNEE,player,user,com);
                updatePlayer(nite::JOINT_LEFT_FOOT,player,user,com);
                updatePlayer(nite::JOINT_RIGHT_FOOT,player,user,com);
                
                int activeJoints=player.size()-1;
                com[0]/=activeJoints;
                com[1]/=activeJoints;
                com[2]/=activeJoints;

                float u,v;
                userTracker.convertJointCoordinatesToDepth(com[0], com[1], com[2], &u, &v);
                
                Bottle &joints=player.addList();
                joints.addString(KINECT_TAGS_BODYPART_COM);
                Bottle &limb=joints.addList();
                limb.addInt((int)u);
                limb.addInt((int)v);
                limb.addDouble(com[0]);
                limb.addDouble(com[1]);
                limb.addDouble(com[2]);
            }
        }
        if(isTracking)
        {
            //The stop tracking procedure is quite slow; we cannot allow the tracking to send inconsistent data to the
            //OPC; if the confidence of all the joints is lower than 0.5, we assume that the tracker actually is not
            //tracking anything, so the only joint written is CoM. In this case we do not send skeleton information.
            if((bones.get(0).asList()->get(1).asList()->get(0).asString()!=KINECT_TAGS_BODYPART_COM));
                *skeleton=bones;
        }
        return true;
    }
    return false;
}

/************************************************************************/
void KinectDriverOpenNI::updatePlayer(nite::JointType type, Bottle &player, nite::UserData& user, yarp::sig::Vector &com)
{
    std::string jName = jointNameAssociation(type);
    const nite::SkeletonJoint& joint = user.getSkeleton().getJoint(type);
    if (joint.getPositionConfidence() > 0.5)
    {
        Bottle &joints=player.addList();
        joints.addString(jName.c_str());
        Bottle &limb=joints.addList();
        com[0]+=joint.getPosition().x/1000;
        com[1]+=joint.getPosition().y/1000;
        com[2]+=joint.getPosition().z/1000;
        float u,v;
        userTracker.convertJointCoordinatesToDepth(joint.getPosition().x, joint.getPosition().y, joint.getPosition().z, &u, &v);
        //kinect with openni does not support 320x240 depth resolution, but we
        //need to send 320x240 depth images to avoid bandwidth problems, so
        //the x and y coordinates are divided by 2 to fit in the 320x240 image.
        if (device==KINECT_TAGS_DEVICE_KINECT)
        {
            limb.addInt((int)u/2);
            limb.addInt((int)v/2);
        }
        else
        {
            limb.addInt((int)u);
            limb.addInt((int)v);                           
        }
        //OpenNI returns millimiters, we want meters
        limb.addDouble(joint.getPosition().x/1000);
        limb.addDouble(joint.getPosition().y/1000);
        limb.addDouble(joint.getPosition().z/1000);
    }
}

/************************************************************************/
bool KinectDriverOpenNI::close()
{
    cvReleaseImage(&depthTmp);
    cvReleaseImage(&depthImage);
    cvReleaseImageHeader(&rgb_big);
    cvReleaseMat(&depthMat);

    cvDestroyAllWindows();

    depthStream.destroy();
    imageStream.destroy();
    openni::OpenNI::shutdown();
    nite::NiTE::shutdown();    

    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::testRetVal(openni::Status nRetVal, string message)
{
    if (nRetVal!=openni::STATUS_OK)
    {
        fprintf(stdout, "%s failed\n", message.c_str());
        return false;
    }
    return true;
}

/************************************************************************/
void KinectDriverOpenNI::update()
{
    depthStream.readFrame(&depthFrame);
    if (imageStream.isValid())
        imageStream.readFrame(&imageFrame);
    if (userTracker.isValid())
        userTracker.readFrame(&userFrame);
}

/************************************************************************/
string KinectDriverOpenNI::jointNameAssociation(nite::JointType joint)
{
    string jointName;
    switch (joint)
    {
        case nite::JOINT_HEAD:
            jointName=KINECT_TAGS_BODYPART_HEAD;
            break;
        case nite::JOINT_LEFT_HAND:
            jointName=KINECT_TAGS_BODYPART_HAND_L;
            break;
        case nite::JOINT_RIGHT_HAND:
            jointName=KINECT_TAGS_BODYPART_HAND_R;
            break;
        case nite::JOINT_LEFT_WRIST:
            jointName=KINECT_TAGS_BODYPART_WRIST_L;
            break;
        case nite::JOINT_RIGHT_WRIST:
            jointName=KINECT_TAGS_BODYPART_WRIST_R;
            break;
        case nite::JOINT_LEFT_ELBOW:
            jointName=KINECT_TAGS_BODYPART_ELBOW_L;
            break;
        case nite::JOINT_RIGHT_ELBOW:
            jointName=KINECT_TAGS_BODYPART_ELBOW_R;
            break;
        case nite::JOINT_NECK:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_C;
            break;
        case nite::JOINT_LEFT_SHOULDER:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_L;
            break;
        case nite::JOINT_RIGHT_SHOULDER:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_R;
            break;
        case nite::JOINT_TORSO:
            jointName=KINECT_TAGS_BODYPART_SPINE;
            break;
        case nite::JOINT_WAIST:
            jointName=KINECT_TAGS_BODYPART_HIP_C;
            break;
        case nite::JOINT_LEFT_HIP:
            jointName=KINECT_TAGS_BODYPART_HIP_L;
            break;
        case nite::JOINT_RIGHT_HIP:
            jointName=KINECT_TAGS_BODYPART_HIP_R;
            break;
        case nite::JOINT_LEFT_KNEE:
            jointName=KINECT_TAGS_BODYPART_KNEE_L;
            break;
        case nite::JOINT_RIGHT_KNEE:
            jointName=KINECT_TAGS_BODYPART_KNEE_R;
            break;
        case nite::JOINT_LEFT_ANKLE :
            jointName=KINECT_TAGS_BODYPART_ANKLE_L;
            break;
        case nite::JOINT_RIGHT_ANKLE :
            jointName=KINECT_TAGS_BODYPART_ANKLE_R;
            break;
        case nite::JOINT_LEFT_FOOT:
            jointName=KINECT_TAGS_BODYPART_FOOT_L;
            break;
        case nite::JOINT_RIGHT_FOOT:
            jointName=KINECT_TAGS_BODYPART_FOOT_R;
            break;
        case nite::JOINT_LEFT_COLLAR:
            jointName=KINECT_TAGS_BODYPART_COLLAR_L;
            break;
        case nite::JOINT_RIGHT_COLLAR:
            jointName=KINECT_TAGS_BODYPART_COLLAR_R;
            break;
        case nite::JOINT_LEFT_FINGERTIP:
            jointName=KINECT_TAGS_BODYPART_FT_L;
            break;
        case nite::JOINT_RIGHT_FINGERTIP:
            jointName=KINECT_TAGS_BODYPART_FT_R;
            break;
    }
    return jointName;
}

/************************************************************************/
bool KinectDriverOpenNI::get3DPoint(int u, int v, yarp::sig::Vector &point3D)
{
    const XnDepthPixel* pDepthMap = depthGenerator.GetDepthMap();
    XnPoint3D p2D, p3D;
    int newU=u;
    int newV=v;
    //request arrives with respect to the 320x240 image, but the depth by default
    // is 640x480 (we resize it before send it to the server)
    if (device==KINECT_TAGS_DEVICE_KINECT)
    {
        newU=u*2;
        newV=v*2;
    }

    p2D.X = newU;
    p2D.Y = newV;
    p2D.Z = pDepthMap[newV*this->def_depth_width+newU];
    depthGenerator.ConvertProjectiveToRealWorld(1, &p2D, &p3D);

    //We provide the 3D point in meters
    point3D.resize(3,0.0);
    point3D[0]=p3D.X/1000;
    point3D[1]=p3D.Y/1000;
    point3D[2]=p3D.Z/1000;
    
    return true;
}

/************************************************************************/
void KinectDriverOpenNI::resizeImage(IplImage* depthTmp, IplImage* depthImage)
{
    for (int i=0; i<depthTmp->height; i++)
    {
        if (i%2==0)
            continue;
        for (int j=0; j<depthTmp->width; j++)
        {
            if (j%2==0)
                continue;
            cvSet2D(depthImage,i/2,j/2,cvGet2D(depthTmp,i,j));
        }
    }
}


