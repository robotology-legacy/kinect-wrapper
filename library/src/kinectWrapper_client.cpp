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
#include <iterator>
#include <yarp/os/Network.h>
#include <kinectWrapper/kinectWrapper_client.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace kinectWrapper;

/************************************************************************/
KinectWrapperClient::KinectWrapperClient()
{
    opening=false;
    verbosity=0;
    init=true;
    remote="";
    local="";
}


/************************************************************************/
KinectWrapperClient::~KinectWrapperClient()
{
    close();
}


/************************************************************************/
int KinectWrapperClient::printMessage(const int level, const char *format, ...) const
{
    if (verbosity>=level)
    {
        fprintf(stdout,"*** %s: ",local.c_str());

        va_list ap;
        va_start(ap,format);
        int ret=vfprintf(stdout,format,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}


/************************************************************************/
bool KinectWrapperClient::open(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    carrier=opt.check("carrier",Value("udp")).asString().c_str();
    verbosity=opt.check("verbosity",Value(0)).asInt();

    if (opt.check("remote"))
        remote=opt.find("remote").asString().c_str();
    else
    {
        printMessage(1,"\"remote\" option is mandatory to open the client!\n");
        return false;
    }

    if (opt.check("local"))
        local=opt.find("local").asString().c_str();
    else
    {
        printMessage(1,"\"local\" option is mandatory to open the client!\n");
        return false;
    }

    depthCV=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    depthCVPl=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    depthFCV=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_32F,1);
    depthFCVPl=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_32F,1);
    playersImage=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_8U,3);
    skeletonImage=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_8U,3);
    depthTmp=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    depthToShow=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_32F,1);

    depthPort.open(("/"+local+"/depth:i").c_str());
    rpc.open(("/"+local+"/rpc").c_str());

    bool ok=true;

    ok&=Network::connect(rpc.getName().c_str(),("/"+remote+"/rpc").c_str());

    if (ok)
    {
        Bottle cmd,reply;
        cmd.addString(KINECT_TAGS_CMD_PING);

        if (rpc.write(cmd,reply))
        {
            if (reply.size()>0)
            {
                if (reply.get(0).asString()==KINECT_TAGS_CMD_ACK)
                {
                    printMessage(1,"successfully connected with the server %s!\n",remote.c_str());

                    info=reply.get(1).asString().c_str();
                    img_width=reply.get(2).asInt();
                    img_height=reply.get(3).asInt();
                    if (reply.get(4).asString()==KINECT_TAGS_SEATED_MODE)
                        seatedMode=true;
                    else
                        seatedMode=false;
                    if (reply.get(5).asString()=="drawAll")
                        drawAll=true;
                    else
                        drawAll=false;
                }
            }
        }
        else
        {
            printMessage(1,"unable to get correct reply from the server %s!\n",remote.c_str());
            close();

            return false;
        }
    }
    else
    {
        printMessage(1,"unable to connect to the server %s!\n",remote.c_str());
        close();

        return false;
    }
    ok=true;
    ok&=Network::connect(("/"+remote+"/depth:o").c_str(),depthPort.getName().c_str(),carrier.c_str());
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        imagePort.open(("/"+local+"/image:i").c_str());
        ok&=Network::connect(("/"+remote+"/image:o").c_str(),imagePort.getName().c_str(),carrier.c_str());
    }
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
    {
        jointsPort.open(("/"+local+"/joints:i").c_str());
        ok&=Network::connect(("/"+remote+"/joints:o").c_str(),jointsPort.getName().c_str(),carrier.c_str());
    }

    if (ok)
        return opening=true;
    else
        return opening=false;
}

/************************************************************************/
void KinectWrapperClient::close()
{
    if (opening)
    {
        depthPort.interrupt();
        rpc.interrupt();

        depthPort.close();
        rpc.close();

        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
        {
            jointsPort.interrupt();
            jointsPort.close();
        }

        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
        {
            imagePort.interrupt();
            imagePort.close();
        }

        cvReleaseImageHeader(&depthCV);
        cvReleaseImageHeader(&depthCVPl);
        cvReleaseImageHeader(&depthFCV);
        cvReleaseImageHeader(&depthFCVPl);
        cvReleaseImage(&playersImage);
        cvReleaseImage(&skeletonImage);
        cvReleaseImage(&depthToShow);

        opening=false;

        printMessage(1,"client closed\n");
    }
    else
        printMessage(3,"client is already closed\n");
}

/************************************************************************/
bool KinectWrapperClient::getDepth(ImageOf<PixelMono16> &depthIm, double *timestamp)
{
    depthIm.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    if (opening)
    {
        ImageOf<PixelMono16>* img;
        if ((img=(ImageOf<PixelMono16>*)depthPort.read(false)))
        {
            Bottle ts;
            depthPort.getEnvelope(ts);
            double timestampD=ts.get(0).asDouble();
            unsigned short* pBuff=(unsigned short*)img->getRawImage();
            for (int i=0; i<img->width()*img->height(); i++)
            {
                //We take only the first 13 bits, that contain the depth value in mm
                unsigned short realDepth = (pBuff[i]&0xFFF8)>>3;
                buf[i]=realDepth;
            }
            cvSetData(depthCV,buf,KINECT_TAGS_DEPTH_WIDTH*2);
            depthIm.wrapIplImage(depthCV);
            if (timestamp!=NULL)
                timestamp=&timestampD;
            return true;
        }
        else
            return false;
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getDepth(ImageOf<PixelFloat> &depthIm, double *timestamp)
{
    depthIm.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    if (opening)
    {
        ImageOf<PixelMono16>* img;
        if ((img=(ImageOf<PixelMono16>*)depthPort.read(false)))
        {
            Bottle ts;
            depthPort.getEnvelope(ts);
            double timestampD=ts.get(0).asDouble();
            unsigned short* pBuff=(unsigned short*)img->getRawImage();
            for (int i=0; i<img->width()*img->height(); i++)
            {
                //We take only the first 13 bits, that contain the depth value in mm
                unsigned short realDepth = (pBuff[i]&0xFFF8);
                float scale=((1.0*realDepth/0xFFF8));
                bufF[i]=scale;
            }
            cvSetData(depthFCV,bufF,KINECT_TAGS_DEPTH_WIDTH*4);
            depthIm.wrapIplImage(depthFCV);
            if (timestamp!=NULL)
                timestamp=&timestampD;
            return true;
        }
        else
            return false;
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getRgb(ImageOf<PixelRgb> &rgbIm, double *timestamp)
{
    if (opening)
    {
        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
        {
            ImageOf<PixelRgb> *tmp;
            if ((tmp=imagePort.read(false)))
            {
                rgbIm=*tmp;
                Bottle ts;
                imagePort.getEnvelope(ts);
                double timestampI=ts.get(0).asDouble();
                if (timestamp!=NULL)
                    timestamp=&timestampI;
                return true;
            }
            else
                return false;
        }
        else
        {
            printMessage(0,"Server does not provide rgb information in this configuration\n");
            return false;
        }
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getPlayers(Matrix &players, double *timestamp)
{
    if (opening)
    {
        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS || info==KINECT_TAGS_DEPTH_PLAYERS)
        {
            players.resize(KINECT_TAGS_DEPTH_HEIGHT,KINECT_TAGS_DEPTH_WIDTH);
            ImageOf<PixelMono16>* img;
            if ((img=(ImageOf<PixelMono16>*)depthPort.read(false)))
            {
                Bottle ts;
                depthPort.getEnvelope(ts);
                double timestampD=ts.get(0).asDouble();
                unsigned short* pBuff=(unsigned short*)img->getRawImage();
                int m=0;
                int n=0;
                for (int i=0; i<img->width()*img->height(); i++)
                {
                    int p=(pBuff[i]&0x0007);
                    if (n==img->width())
                    {
                        n=0;
                        m++;
                    }
                    players(m,n)=p;
                    n++;
                }
                if (timestamp!=NULL)
                    timestamp=&timestampD;
                return true;
            }
            else
                return false;
        }
        else
        {
            printMessage(0,"Server does not provide players information in this configuration\n");
            return false;
        }
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getDepthAndPlayers(ImageOf<PixelMono16> &depthIm, Matrix &players, double *timestamp)
{
    if (opening)
    {
        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS || info==KINECT_TAGS_DEPTH_PLAYERS)
        {
            players.resize(KINECT_TAGS_DEPTH_HEIGHT,KINECT_TAGS_DEPTH_WIDTH);
            ImageOf<PixelMono16>* img;
            if ((img=(ImageOf<PixelMono16>*)depthPort.read(false)))
            {
                Bottle ts;
                depthPort.getEnvelope(ts);
                double timestampD=ts.get(0).asDouble();
                unsigned short* pBuff=(unsigned short*)img->getRawImage();
                int m=0;
                int n=0;
                for (int i=0; i<img->width()*img->height(); i++)
                {
                    unsigned short realDepth = (pBuff[i]&0xFFF8)>>3;
                    bufPl[i]=realDepth;
                    int p=(pBuff[i]&0x0007);
                    if (n==img->width())
                    {
                        n=0;
                        m++;
                    }
                    players(m,n)=p;
                    n++;
                }
                cvSetData(depthCVPl,bufPl,KINECT_TAGS_DEPTH_WIDTH*2);
                depthIm.wrapIplImage(depthCVPl);
                if (timestamp!=NULL)
                    timestamp=&timestampD;
                return true;
            }
            else
                return false;
        }
        else
        {
            printMessage(0,"Server does not provide players information in this configuration\n");
            return false;
        }
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getDepthAndPlayers(ImageOf<PixelFloat> &depthIm, Matrix &players, double *timestamp)
{
    if (opening)
    {
        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS || info==KINECT_TAGS_DEPTH_PLAYERS)
        {
            players.resize(KINECT_TAGS_DEPTH_HEIGHT,KINECT_TAGS_DEPTH_WIDTH);
            ImageOf<PixelMono16>* img;
            if ((img=(ImageOf<PixelMono16>*)depthPort.read(false)))
            {
                Bottle ts;
                depthPort.getEnvelope(ts);
                double timestampD=ts.get(0).asDouble();
                unsigned short* pBuff=(unsigned short*)img->getRawImage();
                int m=0;
                int n=0;
                for (int i=0; i<img->width()*img->height(); i++)
                {
                    unsigned short realDepth = (pBuff[i]&0xFFF8);
                    float scale=((1.0*realDepth/0xFFF8));
                    bufFPl[i]=scale;
                    int p=(pBuff[i]&0x0007);
                    if (n==img->width())
                    {
                        n=0;
                        m++;
                    }
                    players(m,n)=p;
                    n++;
                }
                cvSetData(depthFCVPl,bufFPl,KINECT_TAGS_DEPTH_WIDTH*4);
                depthIm.wrapIplImage(depthFCVPl);
                if (timestamp!=NULL)
                    timestamp=&timestampD;
                return true;
            }
            else
                return false;
        }
        else
        {
            printMessage(0,"Server does not provide players information in this configuration\n");
            return false;
        }
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getJoints(deque<Player> &joints, double *timestamp)
{
    if (opening)
    {
        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
        {
            Bottle* skeleton;
            if ((skeleton=(Bottle*)jointsPort.read(false)))
            {
                if (!(skeleton->size()>0))
                    return false;
                Bottle ts;
                jointsPort.getEnvelope(ts);
                double timestampS=ts.get(0).asDouble();
                if (timestamp!=NULL)
                    timestamp=&timestampS;
                joints=getJoints(skeleton);
                if (joints.size()>0)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
        else
        {
            printMessage(0,"Server does not provide joint information in this configuration\n");
            return false;
        }
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getJoints(Player &joints, int player, double *timestamp)
{
    if (opening)
    {
        if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
        {
            Bottle* skeleton;
            if ((skeleton=(Bottle*)jointsPort.read(false)))
            {
                if (!(skeleton->size()>0))
                    return false;
                Bottle ts;
                jointsPort.getEnvelope(ts);
                double timestampS=ts.get(0).asDouble();
                if (timestamp!=NULL)
                    timestamp=&timestampS;
                joints=getJoints(skeleton,player);
                if (joints.ID==-1)
                    return false;
                else
                    return true;
            }
            else
                return false;
        }
        else
        {
            printMessage(0,"Server does not provide joint information in this configuration\n");
            return false;
        }
    }
    else
    {
        printMessage(1,"client is not open\n");
        return false;
    }
}

/************************************************************************/
bool KinectWrapperClient::getInfo(Property &opt)
{
    if (opening)
    {
        opt.put("info",info.c_str());
        opt.put("img_width",img_width);
        opt.put("img_height",img_height);
        opt.put("depth_width",KINECT_TAGS_DEPTH_WIDTH);
        opt.put("depth_height",KINECT_TAGS_DEPTH_HEIGHT);
        opt.put("seated_mode",(seatedMode?"on":"off"));
        return true;
    }
    return false;
}

/************************************************************************/
std::deque<Player> KinectWrapperClient::getJoints(Bottle* skeleton)
{
    deque<Player> players;
    for (int i=0; i<skeleton->size(); i++)
    {
        Skeleton limbs;
        Bottle* player=skeleton->get(i).asList();
        Player pi;
        pi.ID=player->get(0).asInt();
        for (int j=1; j<player->size(); j++)
        {
            Bottle* joints=player->get(j).asList();
            Bottle* jointsPosition=joints->get(1).asList();
            Joint joint;
            joint.u=jointsPosition->get(0).asInt();
            joint.v=jointsPosition->get(1).asInt();
            joint.x=jointsPosition->get(2).asDouble();
            joint.y=jointsPosition->get(3).asDouble();
            joint.z=jointsPosition->get(4).asDouble();
            limbs[joints->get(0).asString().c_str()]=joint;
        }
        pi.skeleton=limbs;
        players.push_back(pi);
    }
    return players;
}

/************************************************************************/
Player KinectWrapperClient::getJoints(Bottle* skeleton, int playerId)
{
    Player p;
    bool found=false;
    if (playerId<0)
        p=managePlayerRequest(skeleton,playerId);
    else
    {
        for (int i=0; i<skeleton->size(); i++)
        {
            Bottle* player=skeleton->get(i).asList();
            if(player->get(0).asInt()==playerId)
            {
                found=true;
                p.ID=playerId;
                for (int j=1; j<player->size(); j++)
                {
                    Bottle* joints=player->get(j).asList();
                    Bottle* jointsPosition=joints->get(1).asList();
                    Joint joint;
                    joint.u=jointsPosition->get(0).asInt();
                    joint.v=jointsPosition->get(1).asInt();
                    joint.x=jointsPosition->get(2).asDouble();
                    joint.y=jointsPosition->get(3).asDouble();
                    joint.z=jointsPosition->get(4).asDouble();
                    p.skeleton[joints->get(0).asString().c_str()]=joint;
                }
            }
        }
        if (!found)
            p.ID=-1;
    }
    return p;
}

/************************************************************************/
Player KinectWrapperClient::managePlayerRequest(Bottle* skeleton, int playerId)
{
    Player p;
    if (playerId==KINECT_TAGS_CLOSEST_PLAYER)
    {
        double distance=20000;
        deque<Player> players=getJoints(skeleton);
        if (players.size()==0)
        {
            p.ID=-1;
            return p;
        }
        for (unsigned int i=0; i<players.size(); i++)
        {
            if (players.at(i).skeleton[KINECT_TAGS_BODYPART_SHOULDER_C].z < distance)
            {
                distance=players.at(i).skeleton[KINECT_TAGS_BODYPART_SHOULDER_C].z;
                p=players.at(i);
            }
        }
    }
    return p;
}

/************************************************************************/
void KinectWrapperClient::getPlayersImage(const yarp::sig::Matrix &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image)
{
    image.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    for (int i=0; i<KINECT_TAGS_DEPTH_WIDTH; i++)
    {
        for (int j=0; j<KINECT_TAGS_DEPTH_HEIGHT; j++)
        {
            if (players(j,i)==1)
                cvSet2D(playersImage,j,i,cvScalar(255,0,0,0));
            else if (players(j,i)==2)
                cvSet2D(playersImage,j,i,cvScalar(0,255,0,0));
            else if (players(j,i)==3)
                cvSet2D(playersImage,j,i,cvScalar(0,0,255,0));
            else if (players(j,i)==4)
                cvSet2D(playersImage,j,i,cvScalar(255,255,0,0));
            else if (players(j,i)==5)
                cvSet2D(playersImage,j,i,cvScalar(0,255,255,0));
            else if (players(j,i)==6)
                cvSet2D(playersImage,j,i,cvScalar(255,0,255,0));
            else
                cvSet2D(playersImage,j,i,cvScalar(0,0,0,0));
        }
    }
    image.wrapIplImage(playersImage);
}

/************************************************************************/
void KinectWrapperClient::getSkeletonImage(const std::deque<Player> &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image)
{
    image.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    cvZero(skeletonImage);
    for (unsigned int i=0; i<players.size(); i++)
    {
        Skeleton jointsMap=players.at(i).skeleton;
        Skeleton::iterator iterator;
        for (iterator=jointsMap.begin(); iterator!=jointsMap.end(); iterator++)
            if (iterator->second.u!=0 && iterator->second.v!=0 && iterator->first!=KINECT_TAGS_BODYPART_COM)
                cvCircle(skeletonImage,cvPoint(iterator->second.u,iterator->second.v),5,CV_RGB(255,0,0),-1);

       drawLimb(jointsMap,KINECT_TAGS_BODYPART_HEAD,KINECT_TAGS_BODYPART_SHOULDER_C);

        if(drawAll)
        {
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HAND_R,KINECT_TAGS_BODYPART_WRIST_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_WRIST_R,KINECT_TAGS_BODYPART_ELBOW_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_ELBOW_L,KINECT_TAGS_BODYPART_WRIST_L);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_WRIST_L,KINECT_TAGS_BODYPART_HAND_L);
        }
        else
        {
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HAND_R,KINECT_TAGS_BODYPART_ELBOW_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HAND_L,KINECT_TAGS_BODYPART_ELBOW_L);
        }

        drawLimb(jointsMap,KINECT_TAGS_BODYPART_ELBOW_R,KINECT_TAGS_BODYPART_SHOULDER_R);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_R,KINECT_TAGS_BODYPART_SHOULDER_C);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_C,KINECT_TAGS_BODYPART_SHOULDER_L);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_L,KINECT_TAGS_BODYPART_ELBOW_L);


        if (!seatedMode)
        {
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_C,KINECT_TAGS_BODYPART_SPINE);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_R,KINECT_TAGS_BODYPART_KNEE_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_L,KINECT_TAGS_BODYPART_KNEE_L);

            if (drawAll)
            {
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_SPINE,KINECT_TAGS_BODYPART_HIP_C);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_C,KINECT_TAGS_BODYPART_HIP_R);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_C,KINECT_TAGS_BODYPART_HIP_L);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_R,KINECT_TAGS_BODYPART_ANKLE_R);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_ANKLE_R,KINECT_TAGS_BODYPART_FOOT_R);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_L,KINECT_TAGS_BODYPART_ANKLE_L);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_ANKLE_L,KINECT_TAGS_BODYPART_FOOT_L);
            }
            else
            {
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_SPINE,KINECT_TAGS_BODYPART_HIP_R);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_SPINE,KINECT_TAGS_BODYPART_HIP_L);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_R,KINECT_TAGS_BODYPART_FOOT_R);
                drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_L,KINECT_TAGS_BODYPART_FOOT_L);
            }
        }
    }
    image.wrapIplImage(skeletonImage);
}

/************************************************************************/
void KinectWrapperClient::getSkeletonImage(const Player &player, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image)
{
    image.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    cvZero(skeletonImage);
    Skeleton jointsMap=player.skeleton;
    Skeleton::iterator iterator;
    for (iterator=jointsMap.begin(); iterator!=jointsMap.end(); iterator++)
        if (iterator->second.u!=0 && iterator->second.v!=0)
            cvCircle(skeletonImage,cvPoint(iterator->second.u,iterator->second.v),5,CV_RGB(255,0,0),-1);

    drawLimb(jointsMap,KINECT_TAGS_BODYPART_HEAD,KINECT_TAGS_BODYPART_SHOULDER_C);

    if(drawAll)
    {
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_HAND_R,KINECT_TAGS_BODYPART_WRIST_R);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_WRIST_R,KINECT_TAGS_BODYPART_ELBOW_R);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_ELBOW_L,KINECT_TAGS_BODYPART_WRIST_L);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_WRIST_L,KINECT_TAGS_BODYPART_HAND_L);
    }
    else
    {
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_HAND_R,KINECT_TAGS_BODYPART_ELBOW_R);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_HAND_L,KINECT_TAGS_BODYPART_ELBOW_L);
    }

    drawLimb(jointsMap,KINECT_TAGS_BODYPART_ELBOW_R,KINECT_TAGS_BODYPART_SHOULDER_R);
    drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_R,KINECT_TAGS_BODYPART_SHOULDER_C);
    drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_C,KINECT_TAGS_BODYPART_SHOULDER_L);
    drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_L,KINECT_TAGS_BODYPART_ELBOW_L);


    if (!seatedMode)
    {
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_SHOULDER_C,KINECT_TAGS_BODYPART_SPINE);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_R,KINECT_TAGS_BODYPART_KNEE_R);
        drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_L,KINECT_TAGS_BODYPART_KNEE_L);

        if (drawAll)
        {
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_SPINE,KINECT_TAGS_BODYPART_HIP_C);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_C,KINECT_TAGS_BODYPART_HIP_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_HIP_C,KINECT_TAGS_BODYPART_HIP_L);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_R,KINECT_TAGS_BODYPART_ANKLE_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_ANKLE_R,KINECT_TAGS_BODYPART_FOOT_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_L,KINECT_TAGS_BODYPART_ANKLE_L);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_ANKLE_L,KINECT_TAGS_BODYPART_FOOT_L);
        }
        else
        {
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_SPINE,KINECT_TAGS_BODYPART_HIP_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_SPINE,KINECT_TAGS_BODYPART_HIP_L);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_R,KINECT_TAGS_BODYPART_FOOT_R);
            drawLimb(jointsMap,KINECT_TAGS_BODYPART_KNEE_L,KINECT_TAGS_BODYPART_FOOT_L);
        }
    }
    image.wrapIplImage(skeletonImage);
}

/************************************************************************/
void KinectWrapperClient::drawLimb(Skeleton &jointsMap, const string &point1, const string &point2)
{
    Skeleton::const_iterator it=jointsMap.find(point1);
    if (it==jointsMap.end())
        return;

    it=jointsMap.find(point2);
    if(it==jointsMap.end())
        return;

    if ((jointsMap[point1].u==0 && jointsMap[point1].v==0) || (jointsMap[point2].u==0 && jointsMap[point2].v==0))
        return;

    cvLine(skeletonImage,cvPoint(jointsMap[point1].u,jointsMap[point1].v),cvPoint(jointsMap[point2].u,jointsMap[point2].v),CV_RGB(0,255,0));
}

/************************************************************************/
void KinectWrapperClient::getDepthImage(const yarp::sig::ImageOf<yarp::sig::PixelMono16> &depth, yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthToDisplay)
{
    depthTmp=(IplImage*)depth.getIplImage();
    double min,max;
    cvMinMaxLoc(depthTmp,&min,&max);
    cvConvertScale(depthTmp,depthToShow,(1.0/max)*255);
    depthToDisplay.wrapIplImage(depthToShow);
}

/************************************************************************/
bool KinectWrapperClient::get3DPoint(int u, int v, yarp::sig::Vector &point3D)
{
    if (opening)
    {
        Bottle cmd,reply;
        cmd.addString(KINECT_TAGS_CMD_GET3DPOINT);
        cmd.addInt(u);
        cmd.addInt(v);

        point3D.resize(3,0.0);

        if (rpc.write(cmd,reply))
        {
            if (reply.size()>0)
            {
                if (reply.get(0).asString()==KINECT_TAGS_CMD_ACK)
                {
                    printMessage(1,"successfully connected with the server %s!\n",remote.c_str());

                    point3D[0]=reply.get(1).asDouble();
                    point3D[1]=reply.get(2).asDouble();
                    point3D[2]=reply.get(3).asDouble();

                    return true;
                }
            }
        }
        printMessage(1,"unable to get correct reply from the server %s!\n",remote.c_str());

        return false;
    }
    else
        return false;
}

/************************************************************************/
bool KinectWrapperClient::isOpen()
{
    return opening;
}


