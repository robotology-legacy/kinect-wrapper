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

//unnamed namespace for OpenNI callbacks
namespace
{

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& userGenerator,XnUserID nId,void* pCookie)
{
    fprintf(stdout, "New user, asking for calibration\n");
    KinectDriverOpenNI* ki = static_cast<KinectDriverOpenNI*>(pCookie);

    if (ki->getRequireCalibrationPose())
    {
        XnChar strPose[20] = "";
        userGenerator.GetSkeletonCap().GetCalibrationPose(strPose);
        userGenerator.GetPoseDetectionCap().StartPoseDetection(strPose, nId);
    }
    else
        userGenerator.GetSkeletonCap().RequestCalibration(nId, true);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& userGenerator,XnUserID nId,void* pCookie)
{

}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability,const XnChar* strPose,XnUserID nId,void* pCookie)
{
    fprintf(stdout, "Pose detected\n");
    capability.StopPoseDetection(nId);
    UserGenerator* userGenerator=(UserGenerator*)pCookie;
    userGenerator->GetSkeletonCap().RequestCalibration(nId, true);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability,XnUserID nId,void* pCookie)
{
    fprintf(stdout, "Calibration starting\n");
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability,XnUserID nId,XnCalibrationStatus eStatus,void* pCookie)
{
    if (eStatus == XN_CALIBRATION_STATUS_OK)
    {
        fprintf(stdout, "Calibration complete, start tracking\n");
        capability.StartTracking(nId);
    }
    else
    {
        UserGenerator* userGenerator=(UserGenerator*)pCookie;
        XnChar strPose[20] = "";
        userGenerator->GetSkeletonCap().GetCalibrationPose(strPose);
        userGenerator->GetPoseDetectionCap().StartPoseDetection(strPose, nId);
    }
}
} //end unnamed namespace

/************************************************************************/
bool KinectDriverOpenNI::getRequireCalibrationPose()
{
    return this->requireCalibrationPose;
}

/************************************************************************/
bool KinectDriverOpenNI::initialize(Property &opt)
{
    this->info=opt.check("info",Value(KINECT_TAGS_ALL_INFO)).asString().c_str();
    this->seatedMode=false;
    this->img_width=opt.check("img_width",Value(320)).asInt();
    this->img_height=opt.check("img_height",Value(240)).asInt();
    this->requireRemapping=opt.check("remap");

    if (opt.find("device").asString()=="kinect")
    {
        device=KINECT_TAGS_DEVICE_KINECT;
        this->def_image_width=640;
        this->def_image_height=480;
        this->def_depth_width=640;
        this->def_depth_height=480;
    }
    else
    {
        device=KINECT_TAGS_DEVICE_XTION;
        this->def_image_width=320;
        this->def_image_height=240;
        this->def_depth_width=320;
        this->def_depth_height=240;
    }

    depthTmp=cvCreateImage(cvSize(def_depth_width,def_depth_height),IPL_DEPTH_16U,1);
    depthImage=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    rgb_big=cvCreateImageHeader(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,3);
    depthMat = cvCreateMat(def_depth_height,def_depth_width,CV_16UC1);

    XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = context.Init();
    if (!testRetVal(nRetVal, "Context initialization"))
        return false;

    context.SetGlobalMirror(true);

    nRetVal = depthGenerator.Create(context);
    if (!testRetVal(nRetVal, "Depth generator"))
        return false;

    XnMapOutputMode mapMode;
    mapMode.nXRes = this->def_depth_width;
    mapMode.nYRes = this->def_depth_height;
    mapMode.nFPS = 30;
    nRetVal = depthGenerator.SetMapOutputMode(mapMode);
    if (!testRetVal(nRetVal, "Depth Output Setting"))
        return false;

    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        nRetVal = imageGenerator.Create(context);
        if (!testRetVal(nRetVal, "Image generator"))
            return false;

        if (requireRemapping && depthGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
            depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
    }

    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS || info==KINECT_TAGS_DEPTH_PLAYERS || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        nRetVal = userGenerator.Create(context);
        if (!testRetVal(nRetVal, "User generator"))
            return false;

        if ((info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS) && !userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
        {
            fprintf(stdout, "Skeleton not supported\n");
            return false;
        }

        userGenerator.GetSkeletonCap().SetSmoothing(0.6f);

        XnCallbackHandle hUserCallbacks,hCalibrationStart,hCalibrationComplete,hPoseDetected;

        nRetVal = userGenerator.RegisterUserCallbacks(User_NewUser,User_LostUser,this,hUserCallbacks);
        if (!testRetVal(nRetVal, "User callbacks"))
            return false;

        nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart,this,hCalibrationStart);
        if (!testRetVal(nRetVal, "Start calibration callbacks"))
            return false;

        nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete,&userGenerator,hCalibrationComplete);
        if(!testRetVal(nRetVal, "Complete calibration callbacks"))
            return false;

        if(userGenerator.GetSkeletonCap().NeedPoseForCalibration())
        {
            this->requireCalibrationPose=true;
            if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
            {
                fprintf(stdout, "Pose required but not supported\n");
                return false;
            }
            nRetVal=userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
            if (!testRetVal(nRetVal, "Register to Pose Detected"))
                return false;
        }
        else
            this->requireCalibrationPose=false;

        nRetVal = userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected,&userGenerator,hPoseDetected);
        if (!testRetVal(nRetVal, "Pose callbacks"))
            return false;

        userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    }

    nRetVal = context.StartGeneratingAll();
    if (!testRetVal(nRetVal, "Generating data"))
        return false;

    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::readDepth(ImageOf<PixelMono16> &depth, double &timestamp)
{
    const XnDepthPixel* pDepthMap = depthGenerator.GetDepthMap();
    int ts=(int)depthGenerator.GetTimestamp();
    timestamp=(double)ts/1000.0;

    SceneMetaData smd;
    userGenerator.GetUserPixels(0,smd);

    for (int y=0; y<this->def_depth_height; y++)
    {
        for(int x=0;x<this->def_depth_width;x++)
        {
            int player=(smd[y * this->def_depth_width + x]);
            int depth=(pDepthMap[y * this->def_depth_width + x]);
            int finalValue=0;
            finalValue|=((depth<<3)&0XFFF8);
            finalValue|=(player&0X0007);
            //if (x==320 && y==240)
             //   fprintf(stdout, "%d %d\n", ((finalValue&0XFFF8)>>3), finalValue&0X0007);
            //We associate the depth to the first 13 bits, using the last 3 for the player identification
            depthMat->data.s[y * this->def_depth_width + x ]=finalValue;
        }
    }

    if (device==KINECT_TAGS_DEVICE_KINECT)
    {
        cvGetImage(depthMat,depthTmp);
        resizeImage(depthTmp,depthImage);
    }
    else
        cvGetImage(depthMat,depthImage);
    depth.wrapIplImage(depthImage);

    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::readRgb(ImageOf<PixelRgb> &rgb, double &timestamp)
{
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
    int ts=(int)userGenerator.GetTimestamp();
    timestamp=(double)ts/1000.0;
    bool isTracking=false;
    if((info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS))
    {
        Bottle bones;
        bones.clear();
        XnUserID aUsers[KINECT_TAGS_MAX_USERS];
        XnUInt16 nUsers;
        userGenerator.GetUsers(aUsers, nUsers);
        for (XnUInt16 i=0; i<nUsers; i++)
        {
            if ((userGenerator.GetSkeletonCap().IsTracking(aUsers[i])==TRUE))
            {
                isTracking=true;
                double comx=0.0;
                double comy=0.0;
                double comz=0.0;
                Bottle &player=bones.addList();
                player.addInt(aUsers[i]);
                XnUInt16 nJoints = KINECT_TAGS_N_JOINTS;
                XnSkeletonJoint pJoints[KINECT_TAGS_N_JOINTS];
                int activeJoints=nJoints;
                userGenerator.GetSkeletonCap().EnumerateActiveJoints(pJoints,nJoints);
                for (int j=0; j<nJoints; j++)
                {
                    XnSkeletonJoint eJoint = pJoints[j];
                    std::string jName = jointNameAssociation(eJoint);
                    XnSkeletonJointPosition joint;
                    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i],eJoint,joint);
                    if (joint.fConfidence > 0.5)
                    {
                        Bottle &joints=player.addList();
                        joints.addString(jName.c_str());
                        Bottle &limb=joints.addList();
                        XnPoint3D p;
                        comx+=joint.position.X/1000;
                        comy+=joint.position.Y/1000;
                        comz+=joint.position.Z/1000;
                        depthGenerator.ConvertRealWorldToProjective(1,&joint.position,&p);
                        //kinect with openni does not support 320x240 depth resolution, but we
                        //need to send 320x240 depth images to avoid bandwidth problems, so
                        //the x and y coordinates are divided by 2 to fit in the 320x240 image.
                        if (device==KINECT_TAGS_DEVICE_KINECT)
                        {
                            limb.addInt((int)p.X/2);
                            limb.addInt((int)p.Y/2);
                        }
                        else
                        {
                            limb.addInt((int)p.X);
                            limb.addInt((int)p.Y);                           
                        }
                        //OpenNI returns millimiters, we want meters
                        limb.addDouble(joint.position.X/1000);
                        limb.addDouble(joint.position.Y/1000);
                        limb.addDouble(joint.position.Z/1000);
                    }
                    else
                        activeJoints-=1;
                }
                XnPoint3D com;
                comx/=activeJoints;
                comy/=activeJoints;
                comz/=activeJoints;
                com.X=comx;
                com.Y=comy;
                com.Z=comz;
                XnPoint3D projective;
                projective.X=0;
                projective.Y=0;
                depthGenerator.ConvertRealWorldToProjective(1,&com,&projective);
                Bottle &joints=player.addList();
                joints.addString(KINECT_TAGS_BODYPART_COM);
                Bottle &limb=joints.addList();
                limb.addInt((int)projective.X);
                limb.addInt((int)projective.Y);
                limb.addDouble(comx);
                limb.addDouble(comy);
                limb.addDouble(comz);
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
bool KinectDriverOpenNI::close()
{
    cvReleaseImage(&depthTmp);
    cvReleaseImage(&depthImage);
    cvReleaseImageHeader(&rgb_big);
    cvReleaseMat(&depthMat);

    cvDestroyAllWindows();

    depthGenerator.Release();
    userGenerator.Release();
    context.Release();

    return true;
}

/************************************************************************/
bool KinectDriverOpenNI::testRetVal(XnStatus nRetVal, string message)
{
    if (nRetVal!=XN_STATUS_OK)
    {
        fprintf(stdout, "%s failed\n", message.c_str());
        return false;
    }
    return true;
}

/************************************************************************/
void KinectDriverOpenNI::update()
{
    context.WaitOneUpdateAll(userGenerator);
}

/************************************************************************/
string KinectDriverOpenNI::jointNameAssociation(XnSkeletonJoint joint)
{
    string jointName;
    switch (joint)
    {
        case XN_SKEL_HEAD:
            jointName=KINECT_TAGS_BODYPART_HEAD;
            break;
        case XN_SKEL_LEFT_HAND:
            jointName=KINECT_TAGS_BODYPART_HAND_L;
            break;
        case XN_SKEL_RIGHT_HAND:
            jointName=KINECT_TAGS_BODYPART_HAND_R;
            break;
        case XN_SKEL_LEFT_WRIST:
            jointName=KINECT_TAGS_BODYPART_WRIST_L;
            break;
        case XN_SKEL_RIGHT_WRIST:
            jointName=KINECT_TAGS_BODYPART_WRIST_R;
            break;
        case XN_SKEL_LEFT_ELBOW:
            jointName=KINECT_TAGS_BODYPART_ELBOW_L;
            break;
        case XN_SKEL_RIGHT_ELBOW:
            jointName=KINECT_TAGS_BODYPART_ELBOW_R;
            break;
        case XN_SKEL_NECK:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_C;
            break;
        case XN_SKEL_LEFT_SHOULDER:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_L;
            break;
        case XN_SKEL_RIGHT_SHOULDER:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_R;
            break;
        case XN_SKEL_TORSO:
            jointName=KINECT_TAGS_BODYPART_SPINE;
            break;
        case XN_SKEL_WAIST:
            jointName=KINECT_TAGS_BODYPART_HIP_C;
            break;
        case XN_SKEL_LEFT_HIP:
            jointName=KINECT_TAGS_BODYPART_HIP_L;
            break;
        case XN_SKEL_RIGHT_HIP:
            jointName=KINECT_TAGS_BODYPART_HIP_R;
            break;
        case XN_SKEL_LEFT_KNEE:
            jointName=KINECT_TAGS_BODYPART_KNEE_L;
            break;
        case XN_SKEL_RIGHT_KNEE:
            jointName=KINECT_TAGS_BODYPART_KNEE_R;
            break;
        case XN_SKEL_LEFT_ANKLE :
            jointName=KINECT_TAGS_BODYPART_ANKLE_L;
            break;
        case XN_SKEL_RIGHT_ANKLE :
            jointName=KINECT_TAGS_BODYPART_ANKLE_R;
            break;
        case XN_SKEL_LEFT_FOOT:
            jointName=KINECT_TAGS_BODYPART_FOOT_L;
            break;
        case XN_SKEL_RIGHT_FOOT:
            jointName=KINECT_TAGS_BODYPART_FOOT_R;
            break;
        case XN_SKEL_LEFT_COLLAR:
            jointName=KINECT_TAGS_BODYPART_COLLAR_L;
            break;
        case XN_SKEL_RIGHT_COLLAR:
            jointName=KINECT_TAGS_BODYPART_COLLAR_R;
            break;
        case XN_SKEL_LEFT_FINGERTIP:
            jointName=KINECT_TAGS_BODYPART_FT_L;
            break;
        case XN_SKEL_RIGHT_FINGERTIP:
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


