/* Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori, Tobias Fischer
 * email:   ilaria.gori@iit.it, t.fischer@imperial.ac.uk
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
\defgroup kinectServer kinectServer

This module runs the server part of the \ref kinectWrapper library.

\section intro_sec Description
A module that runs a \ref kinectWrapperServer in order to take all the info
from a kinect device.

\section lib_sec Libraries
- YARP libraries.
- \ref kinectWrapper library.

\section parameters_sec Parameters
--verbosity \e verbosity
- specify the verbosity level of the server print-outs.

--period \e period
- server thread period in [ms].

--name \e name
- name of the server.

--img_width \e img_width
- width of the rgb image to send.

--img_height \e img_height
- height of the rgb image to send.

--depth_width \e depth_width
- width of the depth image to send.

--depth_height \e depth_height
- height of the depth image to send.

--seatedMode \e seatedMode
- if put inside the options, the kinect device will be opened in seated mode (only if the driver is SDK).

--remap \e remap
- if OpenNI, it provides depth aligned with rgb

--device \e device
- kinect or xtion

\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <kinectWrapper/kinectWrapper_server.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace kinectWrapper;

class KinectServer: public RFModule
{
protected:
    KinectWrapperServer server;

public:

    bool configure(ResourceFinder &rf)
    {
        int verbosity=rf.check("verbosity",Value(0)).asInt();
        int period=rf.check("period",Value(20)).asInt();
        string name=rf.check("name",Value("kinectServer")).asString().c_str();
        int img_width=rf.check("img_width",Value(320)).asInt();
        int img_height=rf.check("img_height",Value(240)).asInt();
        int depth_width=rf.check("depth_width",Value(320)).asInt();
        int depth_height=rf.check("depth_height",Value(240)).asInt();
        string device=rf.check("device",Value("kinect")).asString().c_str();

        Property options;
        options.put("verbosity",verbosity);
        options.put("period",period);
        options.put("name",name.c_str());
        options.put("img_width",img_width);
        options.put("img_height",img_height);
        options.put("depth_width",depth_width);
        options.put("depth_height",depth_height);
        options.put("device",device.c_str());
        if (rf.check("remap"))
            options.put("remap","true");

        return server.open(options);
    }

    bool close()
    {
        server.close();
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }

    bool updateModule()
    {
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
    rf.setDefaultContext("kinectServer");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    KinectServer mod;
    mod.runModule(rf);
    return 0;
}

