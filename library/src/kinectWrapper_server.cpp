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
#include <algorithm>

#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <kinectWrapper/kinectWrapper_server.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace kinectWrapper;

/************************************************************************/
KinectWrapperServer::KinectWrapperServer() : RateThread(20)
{
    opening=false;
    name="";
}


/************************************************************************/
KinectWrapperServer::~KinectWrapperServer()
{
    if (opening)
        close();
}

/************************************************************************/
bool KinectWrapperServer::read(ConnectionReader &connection)
{
    Bottle cmd, reply;
    cmd.read(connection);

    if (cmd.size()==0)
        reply.addString(KINECT_TAGS_CMD_NACK);
    else
    {
        if (cmd.get(0).asString()==KINECT_TAGS_CMD_PING)
        {
            reply.addString(KINECT_TAGS_CMD_ACK);
            reply.addString(info.c_str());
            reply.addInt(img_width);
            reply.addInt(img_height);
            reply.addString(seatedMode?KINECT_TAGS_SEATED_MODE:"null");
            reply.addString(useSDK?"drawAll":"null");
        }
        else if (cmd.get(0).asString()==KINECT_TAGS_CMD_GET3DPOINT)
        {
            int u=cmd.get(1).asInt();
            int v=cmd.get(2).asInt();
            yarp::sig::Vector point3D;
            if (get3DPoint(u,v,point3D))
            {
                reply.addString(KINECT_TAGS_CMD_ACK);
                reply.addDouble(point3D[0]);
                reply.addDouble(point3D[1]);
                reply.addDouble(point3D[2]);
            }
            else
                reply.addString(KINECT_TAGS_CMD_NACK);
        }
    }

    ConnectionWriter *returnToSender=connection.getWriter();
    if (returnToSender!=NULL)
        reply.write(*returnToSender);

    return true;
}

/************************************************************************/
int KinectWrapperServer::printMessage(const int level, const char *format, ...) const
{
    if (verbosity>=level)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

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
bool KinectWrapperServer::open(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    verbosity=opt.check("verbosity",Value(0)).asInt();
    name=opt.check("name",Value("kinectServer")).asString().c_str();
    period=opt.check("period",Value(20)).asInt();
    info=opt.check("info",Value(KINECT_TAGS_ALL_INFO)).asString().c_str();
    seatedMode=opt.check("seatedMode");
    img_width=opt.check("img_width",Value(320)).asInt();
    img_height=opt.check("img_height",Value(240)).asInt();

    buf=new unsigned short[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    bufPl=new unsigned short[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    bufF=new float[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];
    bufFPl=new float[KINECT_TAGS_DEPTH_WIDTH*KINECT_TAGS_DEPTH_HEIGHT];

#ifdef __USE_SDK__
    driver=new KinectDriverSDK();
#endif

#ifdef __USE_OPENNI__
    driver=new KinectDriverOpenNI();
#endif

    if (!driver->initialize(opt))
    {
        fprintf(stdout, "Kinect failed to initialize\n");
        return false;
    }

#ifdef __USE_SDK__
    useSDK=true;
#endif

#ifdef __USE_OPENNI__
    useSDK=false;
#endif

    if (info==KINECT_TAGS_ALL_INFO)
    {
        jointsPort.open(("/"+name+"/joints:o").c_str());
        imagePort.open(("/"+name+"/image:o").c_str());
    }
    else if (info==KINECT_TAGS_DEPTH_JOINTS)
        jointsPort.open(("/"+name+"/joints:o").c_str());
    else if (info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
        imagePort.open(("/"+name+"/image:o").c_str());

    rpc.open(("/"+name+"/rpc").c_str());
    rpc.setReader(*this);
    depthPort.open(("/"+name+"/depth:o").c_str());

    depthCV=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    depthCVPl=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    depthFCV=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_32F,1);
    depthFCVPl=cvCreateImageHeader(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_32F,1);
    playersImage=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_8U,3);
    skeletonImage=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_8U,3);
    depthTmp=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_16U,1);
    depthToShow=cvCreateImage(cvSize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT),IPL_DEPTH_32F,1);

    depth.resize(KINECT_TAGS_DEPTH_WIDTH, KINECT_TAGS_DEPTH_HEIGHT);
    image.resize(img_width, img_height);

    setRate(period);
    start();

    printMessage(1,"server successfully open\n");

    return opening=true;
}

/************************************************************************/
void KinectWrapperServer::threadRelease()
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        imagePort.interrupt();
        imagePort.close();
    }

    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
    {
        jointsPort.interrupt();
        jointsPort.close();
    }

    depthPort.interrupt();
    depthPort.close();

    rpc.interrupt();
    rpc.close();

    cvReleaseImageHeader(&depthCV);
    cvReleaseImageHeader(&depthCVPl);
    cvReleaseImageHeader(&depthFCV);
    cvReleaseImageHeader(&depthFCVPl);
    cvReleaseImage(&playersImage);
    cvReleaseImage(&skeletonImage);
    cvReleaseImage(&depthTmp);
    cvReleaseImage(&depthToShow);

	if (opening)
    {
    	driver->close();
        delete driver;
    }

    delete[] buf;
    delete[] bufPl;
    delete[] bufF;
    delete[] bufFPl;
}

/************************************************************************/
void KinectWrapperServer::close()
{
    if (opening)
    {
        if (isRunning())
            stop();

        opening=false;
    }
    else
        printMessage(3,"server is already closed\n");
}

/************************************************************************/
void KinectWrapperServer::run()
{
    driver->update();
    if (info==KINECT_TAGS_ALL_INFO)
    {
        mutexDepth.wait();
        bool ready=driver->readDepth(depth, timestampD);
        mutexDepth.post();

        mutexRgb.wait();
        ready&=driver->readRgb(image, timestampI);
        mutexRgb.post();

        mutexSkeleton.wait();
        skeleton.clear();
        ready&=driver->readSkeleton(&skeleton, timestampS);
        mutexSkeleton.post();

        if (depthPort.getOutputCount()>0 && ready)
        {
            mutexDepth.wait();
            depthPort.prepare()=depth;
            tsD.update(timestampD);
            depthPort.setEnvelope(tsD);
            depthPort.write();
            mutexDepth.post();
        }

        if (imagePort.getOutputCount()>0 && ready)
        {
            mutexRgb.wait();
            imagePort.prepare()=image;
            tsI.update(timestampI);
            imagePort.setEnvelope(tsI);
            imagePort.write();
            mutexRgb.post();
        }

        if (jointsPort.getOutputCount()>0 && ready)
        {
            mutexSkeleton.wait();
            jointsPort.prepare()=skeleton;
            tsS.update(timestampS);
            jointsPort.setEnvelope(tsS);
            jointsPort.write();
            mutexSkeleton.post();
        }
    }
    else if (info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        mutexDepth.wait();
        bool ready=driver->readDepth(depth, timestampD);
        mutexDepth.post();

        mutexRgb.wait();
        ready&=driver->readRgb(image, timestampI);
        mutexRgb.post();

        if (depthPort.getOutputCount()>0 && ready)
        {
            mutexDepth.wait();
            depthPort.prepare()=depth;
            tsD.update(timestampD);
            depthPort.setEnvelope(tsD);
            depthPort.write();
            mutexDepth.post();
        }

        if (imagePort.getOutputCount()>0 && ready)
        {
            mutexRgb.wait();
            imagePort.prepare()=image;
            tsI.update(timestampI);
            imagePort.setEnvelope(tsI);
            imagePort.write();
            mutexRgb.post();
        }
    }
    else if (info==KINECT_TAGS_DEPTH_JOINTS)
    {
        mutexDepth.wait();
        bool ready=driver->readDepth(depth, timestampD);
        mutexDepth.post();

        mutexSkeleton.wait();
        skeleton.clear();
        ready&=driver->readSkeleton(&skeleton, timestampS);
        mutexSkeleton.post();

        if (depthPort.getOutputCount()>0 && ready)
        {
            mutexDepth.wait();
            depthPort.prepare()=depth;
            tsD.update(timestampD);
            depthPort.setEnvelope(tsD);
            depthPort.write();
            mutexDepth.post();
        }

        if (jointsPort.getOutputCount()>0 && ready)
        {
            mutexSkeleton.wait();
            jointsPort.prepare()=skeleton;
            tsS.update(timestampS);
            jointsPort.setEnvelope(tsS);
            jointsPort.write();
            mutexSkeleton.post();
        }
    }
    else if (info==KINECT_TAGS_DEPTH || info==KINECT_TAGS_DEPTH_PLAYERS)
    {
        mutexDepth.wait();
        bool ready=driver->readDepth(depth, timestampD);
        mutexDepth.post();

        if (depthPort.getOutputCount()>0 && ready)
        {
            mutexDepth.wait();
            depthPort.prepare()=depth;
            tsD.update(timestampD);
            depthPort.setEnvelope(tsD);
            depthPort.write();
            mutexDepth.post();
        }
    }
}

/************************************************************************/
bool KinectWrapperServer::getDepth(ImageOf<PixelMono16> &depthIm, double *timestamp)
{
    depthIm.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    mutexDepth.wait();
    unsigned short* pBuffDepth=(unsigned short*)depth.getRawImage();
    for (int i=0; i<depth.width()*depth.height(); i++)
    {
        unsigned short realDepth = (pBuffDepth[i]&0xFFF8)>>3;
        buf[i]=realDepth;
    }
    if (timestamp!=NULL)
        timestamp=&timestampD;
    mutexDepth.post();
    cvSetData(depthCV,buf,KINECT_TAGS_DEPTH_WIDTH*2);
    depthIm.wrapIplImage(depthCV);
    return true;
}

/************************************************************************/
bool KinectWrapperServer::getDepth(ImageOf<PixelFloat> &depthIm, double *timestamp)
{
    depthIm.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    mutexDepth.wait();
    unsigned short* pBuffDepth=(unsigned short*)depth.getRawImage();
    for (int i=0; i<depth.width()*depth.height(); i++)
    {
        unsigned short realDepth = (pBuffDepth[i]&0xFFF8);
        float scale=((1.0*realDepth/0xFFF8));
        bufF[i]=scale;
    }
    if (timestamp!=NULL)
        timestamp=&timestampD;
    mutexDepth.post();
    cvSetData(depthFCV,bufF,KINECT_TAGS_DEPTH_WIDTH*4);
    depthIm.wrapIplImage(depthFCV);
    return true;
}

/************************************************************************/
bool KinectWrapperServer::getPlayers(Matrix &players, double *timestamp)
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS || info==KINECT_TAGS_DEPTH_PLAYERS)
    {
        players.resize(KINECT_TAGS_DEPTH_HEIGHT,KINECT_TAGS_DEPTH_WIDTH);
        mutexDepth.wait();
        unsigned short* pBuffPlayers=(unsigned short*)depth.getRawImage();
        int m=0;
        int n=0;
        for (int i=0; i<depth.width()*depth.height(); i++)
        {
            int p=(pBuffPlayers[i]&0x0007);
            if (n==depth.width())
            {
                n=0;
                m++;
            }
            players(m,n)=p;
            n++;
        }
        if (timestamp!=NULL)
            timestamp=&timestampD;
        mutexDepth.post();
        return true;
    }
    return false;
}

/************************************************************************/
bool KinectWrapperServer::getDepthAndPlayers(ImageOf<PixelMono16> &depthIm, Matrix &players, double *timestamp)
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS || info==KINECT_TAGS_DEPTH_PLAYERS)
    {
        players.resize(KINECT_TAGS_DEPTH_HEIGHT,KINECT_TAGS_DEPTH_WIDTH);
        mutexDepth.wait();
        unsigned short* pBuff=(unsigned short*)depth.getRawImage();
        int m=0;
        int n=0;
        for (int i=0; i<depth.width()*depth.height(); i++)
        {
            unsigned short realDepth = (pBuff[i]&0xFFF8)>>3;
            bufPl[i]=realDepth;
            int p=(pBuff[i]&0x0007);
            if (n==depth.width())
            {
                n=0;
                m++;
            }
            players(m,n)=p;
            n++;
        }
        if (timestamp!=NULL)
            timestamp=&timestampD;
        mutexDepth.post();
        cvSetData(depthCVPl,bufPl,KINECT_TAGS_DEPTH_WIDTH*2);
        depthIm.wrapIplImage(depthCVPl);
        return true;
    }
    return false;
}

/************************************************************************/
bool KinectWrapperServer::getDepthAndPlayers(ImageOf<PixelFloat> &depthIm, Matrix &players, double *timestamp)
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB_PLAYERS || info==KINECT_TAGS_DEPTH_PLAYERS)
    {
        players.resize(KINECT_TAGS_DEPTH_HEIGHT,KINECT_TAGS_DEPTH_WIDTH);
        mutexDepth.wait();
        unsigned short* pBuff=(unsigned short*)depth.getRawImage();
        int m=0;
        int n=0;
        for (int i=0; i<depth.width()*depth.height(); i++)
        {
            unsigned short realDepth = (pBuff[i]&0xFFF8);
            float scale=((1.0*realDepth/0xFFF8));
            bufFPl[i]=scale;
            int p=(pBuff[i]&0x0007);
            if (n==depth.width())
            {
                n=0;
                m++;
            }
            players(m,n)=p;
            n++;
        }
        if (timestamp!=NULL)
            timestamp=&timestampD;
        mutexDepth.post();
        cvSetData(depthFCVPl,bufFPl,KINECT_TAGS_DEPTH_WIDTH*4);
        depthIm.wrapIplImage(depthFCVPl);
        return true;
    }
    return false;
}

/************************************************************************/
bool KinectWrapperServer::getRgb(yarp::sig::ImageOf<yarp::sig::PixelRgb> &rgbIm, double *timestamp)
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_RGB || info==KINECT_TAGS_DEPTH_RGB_PLAYERS)
    {
        mutexRgb.wait();
        rgbIm=image;
        if (timestamp!=NULL)
            timestamp=&timestampI;
        mutexRgb.post();
        return true;
    }
    return false;
}

/************************************************************************/
bool KinectWrapperServer::getJoints(deque<Player> &joints, double *timestamp)
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
    {
        mutexSkeleton.wait();
        joints=getJoints();
        if (timestamp!=NULL)
            timestamp=&timestampS;
        mutexSkeleton.post();
        if (joints.size()>0)
            return true;
        else
            return false;
    }
    return false;
}

/************************************************************************/
bool KinectWrapperServer::getJoints(Player &joints, int player, double *timestamp)
{
    if (info==KINECT_TAGS_ALL_INFO || info==KINECT_TAGS_DEPTH_JOINTS)
    {
        mutexSkeleton.wait();
        joints=getJoints(player);
        if (timestamp!=NULL)
            timestamp=&timestampS;
        mutexSkeleton.post();
        if (joints.ID==-1)
            return false;
        else
            return true;
    }
    return false;
}

/************************************************************************/
bool KinectWrapperServer::getInfo(Property &opt)
{
    opt.put("info",info.c_str());
    opt.put("img_width",img_width);
    opt.put("img_height",img_height);
    opt.put("depth_width",KINECT_TAGS_DEPTH_WIDTH);
    opt.put("depth_height",KINECT_TAGS_DEPTH_HEIGHT);
    opt.put("seated_mode",(seatedMode?"on":"off"));
    return true;
}

/************************************************************************/
std::deque<Player> KinectWrapperServer::getJoints()
{
    deque<Player> players;
    for (int i=0; i<skeleton.size(); i++)
    {
        Skeleton limbs;
        Bottle* player=skeleton.get(i).asList();
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
Player KinectWrapperServer::getJoints(int playerId)
{
    Player p;
    bool found=false;
    if (playerId<0)
        p=managePlayerRequest(playerId);
    else
    {
        for (int i=0; i<skeleton.size(); i++)
        {
            Bottle* player=skeleton.get(i).asList();
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
Player KinectWrapperServer::managePlayerRequest(int playerId)
{
    Player p;
    if (playerId==KINECT_TAGS_CLOSEST_PLAYER)
    {
        double distance=20000;
        deque<Player> players=getJoints();
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
void KinectWrapperServer::getPlayersImage(const yarp::sig::Matrix &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image)
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
void KinectWrapperServer::getSkeletonImage(const std::deque<Player> &players, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image)
{
    image.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    cvZero(skeletonImage);
    for (unsigned int i=0; i<players.size(); i++)
    {
        Skeleton jointsMap=players.at(i).skeleton;
        Skeleton::iterator iterator;
        for (iterator=jointsMap.begin(); iterator!=jointsMap.end(); iterator++)
            if (iterator->second.u!=0 && iterator->second.v!=0)
                cvCircle(skeletonImage,cvPoint(iterator->second.u,iterator->second.v),5,CV_RGB(255,0,0),-1);

        drawLimb(jointsMap,KINECT_TAGS_BODYPART_HEAD,KINECT_TAGS_BODYPART_SHOULDER_C);

        if(useSDK)
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

            if (useSDK)
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
void KinectWrapperServer::getSkeletonImage(const Player &player, yarp::sig::ImageOf<yarp::sig::PixelBgr> &image)
{
    image.resize(KINECT_TAGS_DEPTH_WIDTH,KINECT_TAGS_DEPTH_HEIGHT);
    cvZero(skeletonImage);
    Skeleton jointsMap=player.skeleton;
    Skeleton::iterator iterator;
    for (iterator=jointsMap.begin(); iterator!=jointsMap.end(); iterator++)
        if (iterator->second.u!=0 && iterator->second.v!=0)
            cvCircle(skeletonImage,cvPoint(iterator->second.u,iterator->second.v),5,CV_RGB(255,0,0),-1);

   drawLimb(jointsMap,KINECT_TAGS_BODYPART_HEAD,KINECT_TAGS_BODYPART_SHOULDER_C);

    if(useSDK)
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

        if (useSDK)
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
void KinectWrapperServer::drawLimb(Skeleton &jointsMap, const string &point1, const string &point2)
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
void KinectWrapperServer::getDepthImage(const yarp::sig::ImageOf<yarp::sig::PixelMono16> &depth, yarp::sig::ImageOf<yarp::sig::PixelFloat> &depthToDisplay)
{
    depthTmp=(IplImage*)depth.getIplImage();
    double min,max;
    cvMinMaxLoc(depthTmp,&min,&max);
    cvConvertScale(depthTmp,depthToShow,(1.0/max)*255);
    depthToDisplay.wrapIplImage(depthToShow);
}

/************************************************************************/
bool KinectWrapperServer::get3DPoint(int u, int v, yarp::sig::Vector &point3D)
{
    driver->get3DPoint(u,v,point3D);
    return true;
}

/************************************************************************/
bool KinectWrapperServer::isOpen()
{
    return opening;
}


