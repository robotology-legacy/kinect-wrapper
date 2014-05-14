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

#include <stdio.h>
#include <stdarg.h>
#include <yarp/os/Network.h>
#include <kinectWrapper/kinectDriverSDK.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace kinectWrapper;

/************************************************************************/
bool KinectDriverSDK::initialize(Property &opt)
{
    this->info=opt.check("info",Value(KINECT_TAGS_ALL_INFO)).asString().c_str();
    this->seatedMode=opt.check("seatedMode");
    this->img_width=opt.check("img_width",Value(320)).asInt();
    this->img_height=opt.check("img_height",Value(240)).asInt();

    this->def_image_width=640;
    this->def_image_height=480;

    buf=new USHORT[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];

    color=cvCreateImageHeader(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,4);
    depthTmp=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);

    initC=false;
    initD=false;

    HRESULT hr;

    if (info==KINECT_TAGS_ALL_INFO)
        hr= NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON| NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |NUI_INITIALIZE_FLAG_USES_COLOR);
    else if (info==KINECT_TAGS_DEPTH_JOINTS)
        hr= NuiInitialize( NUI_INITIALIZE_FLAG_USES_SKELETON| NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
    else if (info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
        hr= NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |NUI_INITIALIZE_FLAG_USES_COLOR);
    else
        hr= NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);

    if( hr != S_OK )
    {
         fprintf(stdout,"NuiInitialize failed\n");
         return false;
    }

    h1 = CreateEvent( NULL, TRUE, FALSE, NULL );
    h2 = NULL;

    if (info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480, 0, 2, h1, &h2);

        if( FAILED( hr ) )
        {
            fprintf(stdout,"Could not open image stream video\n");
            return false;
        }
    }

    h3 = CreateEvent( NULL, TRUE, FALSE, NULL );
    h4 = NULL;

    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, h3, &h4);

    if(NuiImageStreamSetImageFrameFlags(h4, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE )!=S_OK)
        fprintf(stdout,"NO NEAR MODE\n");

    if( FAILED( hr ) )
    {
       fprintf(stdout,"Could not open depth stream video\n");
        return false;
    }

    if(seatedMode && (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS))
        NuiSkeletonTrackingEnable(h3,NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);

    return true;
}

/************************************************************************/
bool KinectDriverSDK::close()
{
    cvReleaseImageHeader(&color);
    cvReleaseImageHeader(&depthTmp);

    cvDestroyAllWindows();

    NuiShutdown();

    delete[] buf;

    return true;
}

/************************************************************************/
int KinectDriverSDK::setColorImg(HANDLE h,IplImage* color,const NUI_IMAGE_FRAME * pImageFrame)
{
    INuiFrameTexture  * pTexture=pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
         BYTE * pBuffer = (BYTE*) LockedRect.pBits;
         cvSetData(color,pBuffer,LockedRect.Pitch);
    }
    return 0;
}

/************************************************************************/
int KinectDriverSDK::setDepthImg(HANDLE h,IplImage* depth, const NUI_IMAGE_FRAME * pImageFrame )
{
    INuiFrameTexture  * pTexture=pImageFrame->pFrameTexture;

    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );

    if( LockedRect.Pitch != 0 )
    {
        USHORT * pBuff = (USHORT*) LockedRect.pBits;

        for(int i=0;i<KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT;i++)
            buf[i]=pBuff[i];

        cvSetData(depth,buf,KINECT_TAGS_DEPTH_WIDTH*2);
    }
    return 0;
}

/************************************************************************/
const NUI_IMAGE_FRAME * KinectDriverSDK::retrieveImg(HANDLE h)
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame( h, 0, &pImageFrame );

    if( FAILED( hr ) )
        return NULL;
    else
        return pImageFrame;
}

/************************************************************************/
bool KinectDriverSDK::readRgb(ImageOf<PixelRgb> &rgb, double &timestamp)
{
    rgb.resize(img_width,img_height);
    const NUI_IMAGE_FRAME *colIm;
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
        colIm=retrieveImg(h2);
    else
        return false;

    if (colIm!=NULL)
    {
        IplImage* rgb_big=cvCreateImage(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,3);
        IplImage* foo=cvCreateImage(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,1);
        IplImage* r=cvCreateImage(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,1);
        IplImage* g=cvCreateImage(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,1);
        IplImage* b=cvCreateImage(cvSize(def_image_width,def_image_height),IPL_DEPTH_8U,1);
        setColorImg(h2,color,colIm);
        cvSplit(color,r,g,b,foo);
        cvMerge(b,g,r,NULL,rgb_big);
        cvResize(rgb_big,(IplImage*)rgb.getIplImage());
        NuiImageStreamReleaseFrame(h2, colIm);
        initC=true;
        timestamp=(double)(colIm->liTimeStamp).QuadPart;
        cvReleaseImage(&rgb_big);
        cvReleaseImage(&foo);
        cvReleaseImage(&r);
        cvReleaseImage(&g);
        cvReleaseImage(&b);
        return true;
    }
    return false;
}

/************************************************************************/
bool KinectDriverSDK::readDepth(ImageOf<PixelMono16> &depth, double &timestamp)
{
    const NUI_IMAGE_FRAME *depthIm=retrieveImg(h4);
    if(depthIm!=NULL)
    {
        setDepthImg(h4,depthTmp,depthIm);
        depth.wrapIplImage(depthTmp);
        NuiImageStreamReleaseFrame(h4, depthIm);
        initD=true;
        timestamp=(double)(depthIm->liTimeStamp).QuadPart;
        return true;
    }
    return false;
}

/************************************************************************/
bool KinectDriverSDK::readSkeleton(Bottle *skeleton, double &timestamp)
{
    skeleton->clear();
    if(initC && initD && (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS))
    {
        NUI_SKELETON_FRAME SkeletonFrame;
        HRESULT hr = NuiSkeletonGetNextFrame( 0, &SkeletonFrame );

        Bottle bones;
        bones.clear();
        int nJointsUsed=NUI_SKELETON_POSITION_COUNT;
        if (seatedMode)
            nJointsUsed=10;
        for(int i = 0; i< NUI_SKELETON_COUNT;i++)
        {
            if(SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED)
            {
                double comx=0.0;
                double comy=0.0;
                double comz=0.0;
                Bottle &player=bones.addList();
                player.addInt(i+1);
                NuiTransformSmooth(&SkeletonFrame,NULL);
                for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
                {
                    if (isJointProvided(j))
                    {
                        Bottle &joints=player.addList();
                        joints.addString(jointNameAssociation(j).c_str());
                        Bottle &joint=joints.addList();
                        float fx,fy;
                        NuiTransformSkeletonToDepthImage(SkeletonFrame.SkeletonData[i].SkeletonPositions[j],&fx,&fy,NUI_IMAGE_RESOLUTION_320x240);
                        joint.addInt((int)fx);
                        joint.addInt((int)fy);
                        joint.addDouble(SkeletonFrame.SkeletonData[i].SkeletonPositions[j].x);
                        joint.addDouble(SkeletonFrame.SkeletonData[i].SkeletonPositions[j].y);
                        joint.addDouble(SkeletonFrame.SkeletonData[i].SkeletonPositions[j].z);
                        comx+=SkeletonFrame.SkeletonData[i].SkeletonPositions[j].x;
                        comy+=SkeletonFrame.SkeletonData[i].SkeletonPositions[j].y;
                        comz+=SkeletonFrame.SkeletonData[i].SkeletonPositions[j].z;
                    }
                }
                comx/=nJointsUsed;
                comy/=nJointsUsed;
                comz/=nJointsUsed;
                Vector4 pos;
                pos.x=comx;
                pos.y=comy;
                pos.z=comz;
                pos.w=1.0;
                float fx,fy;
                NuiTransformSkeletonToDepthImage(pos,&fx,&fy,NUI_IMAGE_RESOLUTION_320x240);
                Bottle &joints=player.addList();
                joints.addString(KINECT_TAGS_BODYPART_COM);
                Bottle &limb=joints.addList();
                limb.addInt((int)fx);
                limb.addInt((int)fy);
                limb.addDouble((double)comx);
                limb.addDouble((double)comy);
                limb.addDouble((double)comz);
            }
        }
        *skeleton=bones;
        initC=initD=false;
        return true;
    }
    return false;
}

/************************************************************************/
string KinectDriverSDK::jointNameAssociation(int id)
{
    string jointName;
    switch (id)
    {
        case NUI_SKELETON_POSITION_HEAD:
            jointName=KINECT_TAGS_BODYPART_HEAD;
            break;
        case NUI_SKELETON_POSITION_HAND_LEFT:
            jointName=KINECT_TAGS_BODYPART_HAND_L;
            break;
        case NUI_SKELETON_POSITION_HAND_RIGHT:
            jointName=KINECT_TAGS_BODYPART_HAND_R;
            break;
        case NUI_SKELETON_POSITION_WRIST_LEFT:
            jointName=KINECT_TAGS_BODYPART_WRIST_L;
            break;
        case NUI_SKELETON_POSITION_WRIST_RIGHT:
            jointName=KINECT_TAGS_BODYPART_WRIST_R;
            break;
        case NUI_SKELETON_POSITION_ELBOW_LEFT:
            jointName=KINECT_TAGS_BODYPART_ELBOW_L;
            break;
        case NUI_SKELETON_POSITION_ELBOW_RIGHT:
            jointName=KINECT_TAGS_BODYPART_ELBOW_R;
            break;
        case NUI_SKELETON_POSITION_SHOULDER_CENTER:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_C;
            break;
        case NUI_SKELETON_POSITION_SHOULDER_LEFT:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_L;
            break;
        case NUI_SKELETON_POSITION_SHOULDER_RIGHT:
            jointName=KINECT_TAGS_BODYPART_SHOULDER_R;
            break;
        case NUI_SKELETON_POSITION_SPINE:
            jointName=KINECT_TAGS_BODYPART_SPINE;
            break;
        case NUI_SKELETON_POSITION_HIP_CENTER:
            jointName=KINECT_TAGS_BODYPART_HIP_C;
            break;
        case NUI_SKELETON_POSITION_HIP_LEFT:
            jointName=KINECT_TAGS_BODYPART_HIP_L;
            break;
        case NUI_SKELETON_POSITION_HIP_RIGHT:
            jointName=KINECT_TAGS_BODYPART_HIP_R;
            break;
        case NUI_SKELETON_POSITION_KNEE_LEFT:
            jointName=KINECT_TAGS_BODYPART_KNEE_L;
            break;
        case NUI_SKELETON_POSITION_KNEE_RIGHT:
            jointName=KINECT_TAGS_BODYPART_KNEE_R;
            break;
        case NUI_SKELETON_POSITION_ANKLE_LEFT:
            jointName=KINECT_TAGS_BODYPART_ANKLE_L;
            break;
        case NUI_SKELETON_POSITION_ANKLE_RIGHT:
            jointName=KINECT_TAGS_BODYPART_ANKLE_R;
            break;
        case NUI_SKELETON_POSITION_FOOT_LEFT:
            jointName=KINECT_TAGS_BODYPART_FOOT_L;
            break;
        case NUI_SKELETON_POSITION_FOOT_RIGHT:
            jointName=KINECT_TAGS_BODYPART_FOOT_R;
            break;
    }
    return jointName;
}

/************************************************************************/
bool KinectDriverSDK::isJointProvided(int id)
{
    bool isProvided;
    if (!seatedMode)
        return true;

    switch (id)
    {
        case NUI_SKELETON_POSITION_HEAD:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_HAND_LEFT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_HAND_RIGHT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_WRIST_LEFT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_WRIST_RIGHT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_ELBOW_LEFT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_ELBOW_RIGHT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_SHOULDER_CENTER:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_SHOULDER_LEFT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_SHOULDER_RIGHT:
            isProvided=true;
            break;
        case NUI_SKELETON_POSITION_SPINE:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_HIP_CENTER:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_HIP_LEFT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_HIP_RIGHT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_KNEE_LEFT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_KNEE_RIGHT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_ANKLE_LEFT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_ANKLE_RIGHT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_FOOT_LEFT:
            isProvided=false;
            break;
        case NUI_SKELETON_POSITION_FOOT_RIGHT:
            isProvided=false;
            break;
    }
    return isProvided;
}

/************************************************************************/
bool KinectDriverSDK::get3DPoint(int u, int v, yarp::sig::Vector &point3D)
{
    CvScalar val= cvGet2D(depthTmp,v,u);
    USHORT depthValue =(USHORT)(val.val[0]);
    Vector4 point = NuiTransformDepthImageToSkeleton(u,v,depthValue,NUI_IMAGE_RESOLUTION_320x240);
    point3D.resize(3,0.0);
    point3D[0]=point.x;
    point3D[1]=point.y;
    point3D[2]=point.z;

    return true;
}

/************************************************************************/
void KinectDriverSDK::update()
{
    //sdk updates one data stream per time
}




