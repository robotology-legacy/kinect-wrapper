kinectWrapper
=============

A C++ library that wraps around Kinect SDK or OpenNI device drivers.

## Introduction

The main purpose of this YARP wrapper is to abstract from the hardware layer and provide data neatly and effectively over the network. The resulting gains are threefold: (1) the benefit in controlling the bandwidth while preventing data duplication; (2) the significant facilitation from user standpoint of writing code by means of proxy access to the hardware, with resort to a standard set of YARP API in place of direct calls to a custom set of Kinect API; (3) the possibility to easily scale up with the addition of future modules that will access Kinect-like devices.

The _Kinect Wrapper_ has been designed and implemented adhering to the client-server paradigm, where the server (i.e. `KinectServer`) takes care of streaming out all the information over YARP ports and the clients (i.e. `KinectClient`) are light YARP front-end instantiated within the user code that read Kinect data from the network and provide them in a convenient format.
To further separate the driver interfacing the Kinect device from the part of the sever dealing with YARP communication, a third abstraction layer has been considered, namely the `KinectDriver`, and located at lowest level in the wrapper hierarchy with the requirement of providing to the `KinectServer` the Kinect raw data to be marshaled and sent over the network. The `KinectDriver` is thus specialized in two implementations: the `KinectDriverSDK` and the `KinectDriverOpenNI`, respectively.
The hierarchical structure of the wrapper can be seen in the following diagram:

![Diagram of Kinect-Wrapper architecture](misc/architecture.png)

In particular, the `KinectServer` is realized as a periodic thread that initially opens the Kinect device using either the `KinectDriverSDK` or the `KinectDriverOpenNI` components; then, at each run, the server reads the information as configured by the user (depth data only, user skeleton only, RGB images only or combination of them) from the driver and sends them over YARP ports. On the client side, the user relies on some simple function calls to retrieve such information.
