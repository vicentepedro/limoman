// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright: (C) 2015 Vis Lab - Instituto Superior Tecnico
// Authors: Lorenzo Jamone (lorejam)
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <iostream>
#include <string>

#include <glove.hpp>

#include <yarp/os/all.h>
 
using namespace std;

using namespace glove::devices;
using namespace glove::devices::local;
using namespace glove::calib;

using namespace yarp::os; 


class GloveToYarpThread : public RateThread
{
   /* class variables */ 
    
    int    periodInMsec;
    double periodInSec;
    int    displayRate;
    int    sendBufferIndex;
    unsigned int gloveBufferId;
    
    int    iteration;
    
    int    nAxes;
    
    bool   read;

    void dumpData();
    void readData();
    void readDataFast();
    void sendData();
    void initData();
    void remapData();
    
    /* thread parameters: they are pointers so that they refer to the original variables in myModule */

    BufferedPort< Bottle > *outPort;
    LocalCyberGlove* glove;

    double* jointsPos;
    double* defJointVals;
    double* tmpJointVals;

    vector<double> sensValues;

    double startRead;
    double readTime;
    double startTime;
    double cycleTime;
    
public: 
  
    int mode; // 0 - idle ; 1 - send ; 2 - dump ; 3 - send & dump

    /* class methods */
    GloveToYarpThread(LocalCyberGlove *g, BufferedPort< Bottle > *p, int per, double *df);

    bool threadInit();     
    void threadRelease();
    void run(); 
    void simControlStep();
};

class UserCmdThread : public Thread
{
    GloveToYarpThread *g2yThread;
    RpcServer *setPort;                        // input/output port that receives commands from user (typically to change some parameters), and sends replies
    
public:
    
    UserCmdThread(GloveToYarpThread *t, RpcServer *s);
    bool threadInit();     
    void threadRelease();
    void run(); 
    void simControlStep();
    
    void checkUserCmd();
  
};


class GloveToYarpModule : public RFModule
{
    /* module parameters */

    string moduleName;
    string moduleLocalPortName; 
 
    string outPortName;
    string setPortName;

    string gloveLocalPortName;
    string gloveCalibFileName;

    Bottle defValsBottle; 
    double *defVals;
    
    double velocity;
    double maxVel;
    int    period;
    int    nAxes;
     
    /* class variables */

    LocalCyberGlove* glove;

    BufferedPort< Bottle > outPort;                // output port that sends hand joints values
    RpcServer setPort;                             // input/output port that receives commands from user (typically to change some parameters), and sends replies

    /* pointer to a new thread to be created and started in configure() and stopped in close() */ 

    GloveToYarpThread    *gloveToYarpThread;
    UserCmdThread        *cmdT;

public:
   
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    double getPeriod(); 
    bool updateModule();
};



