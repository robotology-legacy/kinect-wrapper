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
\defgroup kinectClientExample kinectClientExample

Example module for the use of \ref kinectClientExample "Kinect 
Wrapper Client". 

\section intro_sec Description
This simple module retrieves and display depth and rgb images, players information and the skeleton
reading everything from a \ref kinectServer.

It requires the \ref kinectServer running.

\section lib_sec Libraries
- YARP libraries.
- \ref kinectWrapper library.

\section parameters_sec Parameters
--verbosity \e verbosity
- specify the verbosity level of the client print-outs.

--carrier \e carrier
- specify the protocol used to connect to the server ports.

--remote \e remote
- specify the kinectServer name to connect to.

--local \e name
- specify the kinectClient stem-name.

\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <kinectWrapper/kinectTags.h>
#include <kinectWrapper/kinectWrapper_client.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace kinectWrapper;

class KinectClient: public RFModule
{
protected:
    KinectWrapperClient client;
    ImageOf<PixelRgb> rgb;
    ImageOf<PixelMono16> depth;
    ImageOf<PixelFloat> depthToDisplay;
    ImageOf<PixelBgr> playersImage;
    ImageOf<PixelBgr> skeletonImage;
    IplImage* depthTmp;
    IplImage* rgbTmp;
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > depthPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > playersPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > skeletonPort;
    
    bool showImages;
    Player joint;
    deque<Player> joints;
    Matrix players;

public:

    bool configure(ResourceFinder &rf)
    {
        int verbosity=rf.check("verbosity",Value(0)).asInt();
        string name=rf.check("name",Value("kinectClientExample")).asString().c_str();
        string show=rf.check("showImages",Value("false")).asString().c_str();
        showImages=(show=="true");
        
        depthPort.open("/"+name+"/depthPort:o");
        imagePort.open("/"+name+"/imagePort:o");
        playersPort.open("/"+name+"/playersPort:o");
        skeletonPort.open("/"+name+"/skeletonPort:o");

        Property options;
        options.put("carrier","udp");
        options.put("remote","kinectServer");
        options.put("local",(name+"/client").c_str());
        options.put("verbosity",verbosity);

        if (!client.open(options))
            return false;

        Property opt;
        client.getInfo(opt);

        int img_width=opt.find("img_width").asInt();
        int img_height=opt.find("img_height").asInt();
        int depth_width=opt.find("depth_width").asInt();
        int depth_height=opt.find("depth_height").asInt();
        rgb.resize(img_width, img_height);
        depth.resize(depth_width,depth_height);
        depthToDisplay.resize(depth_width,depth_height);
        playersImage.resize(depth_width,depth_height);
        skeletonImage.resize(depth_width,depth_height);
        
        depthTmp=cvCreateImage(cvSize(depth_width,depth_height),IPL_DEPTH_32F,1);
        rgbTmp=cvCreateImage(cvSize(img_width,img_height),IPL_DEPTH_8U,3);
        
        if (showImages)
        {
            cvNamedWindow("rgb",CV_WINDOW_AUTOSIZE);
            cvMoveWindow("rgb", 160, 100);
            cvNamedWindow("depth",CV_WINDOW_AUTOSIZE);
            cvMoveWindow("depth", 510, 100);
            cvNamedWindow("skeleton", CV_WINDOW_AUTOSIZE);
            cvMoveWindow("skeleton", 860, 100);
            cvNamedWindow("players", CV_WINDOW_AUTOSIZE);
            cvMoveWindow("players", 320, 400);
        }

        return true;
    }

    bool close()
    {
        depthPort.interrupt();
        depthPort.close();
        imagePort.interrupt();
        imagePort.close();
        playersPort.interrupt();
        playersPort.close();
        skeletonPort.interrupt();
        skeletonPort.close();
        client.close();
        cvReleaseImage(&depthTmp);
        cvReleaseImage(&rgbTmp);
        return true;
    }

    double getPeriod()
    {
        return 0.01;
    }

    bool updateModule()
    {
        client.getDepthAndPlayers(depth,players);
        client.getRgb(rgb);

        /*Alternatively you can ask for getJoints(joints), retrieving a deque of Players,
        having information on all the players instead of having only information on
        the closest one*/
        bool tracked=client.getJoints(joint,KINECT_TAGS_CLOSEST_PLAYER);

        if (tracked)
            client.getSkeletonImage(joint,skeletonImage);

        client.getPlayersImage(players,playersImage);
        client.getDepthImage(depth,depthToDisplay);
        
        if (depthPort.getOutputCount()>0)
        {
            depthPort.prepare()=depthToDisplay;
            depthPort.write();
        }
        
        if (imagePort.getOutputCount()>0)
        {
            imagePort.prepare()=rgb;
            imagePort.write();
        }
        
        if (playersPort.getOutputCount()>0)
        {
            playersPort.prepare()=playersImage;
            playersPort.write();
        }
        
        if (skeletonPort.getOutputCount()>0)
        {
            skeletonPort.prepare()=skeletonImage;
            skeletonPort.write();
        }
        
        int u=160;
        int v=120;
        yarp::sig::Vector point3D;
        client.get3DPoint(u,v,point3D);
        
        fprintf(stdout, "%s\n", point3D.toString().c_str());

        if (showImages)
        {
            cvConvertScale((IplImage*)depthToDisplay.getIplImage(),depthTmp,1.0/255);
            cvShowImage("depth",depthTmp);
            cvShowImage("players",(IplImage*)playersImage.getIplImage());
            cvShowImage("skeleton",(IplImage*)skeletonImage.getIplImage());

            cvWaitKey(1);

            cvCvtColor((IplImage*)rgb.getIplImage(),rgbTmp,CV_BGR2RGB);
            cvShowImage("rgb",rgbTmp);
        }

        return true;
    }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout, "Yarp network not available\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("kinectClientExample");
    rf.configure(argc,argv);

    KinectClient mod;
    mod.runModule(rf);
    return 0;
}

