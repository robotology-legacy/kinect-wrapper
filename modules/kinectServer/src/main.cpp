/*
 * Copyright (C) 2012 EFAA Consortium, European Commission FP7 Project IST-270490
 * Authors: Ilaria Gori
 * email:   ilaria.gori@iit.it
 * website: http://efaa.upf.edu/
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * $EFAA_ROOT/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
\defgroup kinectServer pmpActionsModule

@ingroup efaa_modules

This module runs the server part of the \ref kinectWrapper library.

\section intro_sec Description
A module that runs a \ref kinectWrapperServer in order to take all the info
from a kinect device.

\section lib_sec Libraries
- YARP libraries.
- \ref kinect kinect library.

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

#include <efaa/kinect/kinectWrapper_server.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace efaa::kinect;

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
        string device=rf.check("device",Value("kinect")).asString().c_str();

        Property options;
        options.put("verbosity",verbosity);
        options.put("period",period);
        options.put("name",name.c_str());
        options.put("img_width",img_width);
        options.put("img_height",img_height);
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
        return 0.01;
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
    rf.setDefaultContext("kinectServer/conf");
    rf.setDefault("from","kinect.ini");
    rf.configure("EFAA_ROOT",argc,argv);

    KinectServer mod;

    mod.runModule(rf);

    return 0;
}

