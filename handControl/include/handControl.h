// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


#ifndef __ICUB_HANDCONTROL_MODULE_H__
#define __ICUB_HANDCONTROL_MODULE_H__ 

#include <iostream>
#include <string>
#include <vector>

#include <math.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
 
using namespace std;

using namespace yarp;
using namespace yarp::os; 
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::sig;


class HandControlThread : public RateThread
{
   /* class variables */ 

    IPositionControl2 *pos;
    IPositionDirect *posDir;
    IControlMode2 *ctrlMode;
    IVelocityControl *vel;
    IEncoders *encs;
    IControlLimits *limits;
    
    Vector encoders;
    Vector v_estimate;
    Vector p_command;
    Vector v_command;
    
    Matrix posRef;
    Matrix gloveData;
    Matrix jointsMap;
    Matrix jointsGain;
    
    Matrix handJointsPos;
    
    int    nAxes;
    int    ctrlJoints;
    int    gloveDataDim;
    
    int    ctrlStep;
    int    periodInMsec;
    double periodInSec;
    
    bool   motionStarted;
    bool   motionDone;
    
    bool readData();
    
    void readEncs(bool v=true);
    void dumpData();
    void handPosMove();
    void handPosMoveDirect();
    void controlStep();
    void updateRef();    
    
    /* thread parameters: they are pointers so that they refer to the original variables in myModule */

    PolyDriver             *robotDevice;
    BufferedPort< Bottle > *inputPort;
    Vector *jointLimitsMin;
    Vector *jointLimitsMax;
    Vector *handRestPos;
    
    Vector *controlledJointsVector; 
    
    double *handPosRef;
    double *handVelRef;
    int *controlledJoints;
    int *controlModes;
    
    double velocity;
    double refSpeed;
    double startT;
    double moveT;
    double cycleTime;
    double maxV;
    
public: 
  
    int controlMode; // 0 - idle; 1 - position direct; 2 - postion control with bell-shaped trajectories

    /* class methods */
    HandControlThread(PolyDriver *robDev, Vector *jLimitsMin, Vector *jLimitsMax, Vector *hRestPos, double vel, double mV, int per, BufferedPort< Bottle > *inPort, Vector *cJoints);
    bool threadInit();     
    void threadRelease();
    void run(); 
    void simControlStep();
};

class UserCmdThread : public Thread
{
    HandControlThread *handConThread;
    RpcServer *setPort;                        // input/output port that receives commands from user (typically to change some parameters), and sends replies
    
public:
    
    UserCmdThread(HandControlThread *hcThread, RpcServer *sPort );
    bool threadInit();     
    void threadRelease();
    void run(); 
    void simControlStep();
    
    void checkUserCmd();
  
};


class HandControlModule : public RFModule
{
    /* module parameters */

    string moduleName;
    string robotName;
    string partName;

    string remotePortName;  
    string localPortName;  
    
    string setPortName;
    string inputPortName;
    
    string deviceName;
    
    Vector jointLimitsMin;
    Vector jointLimitsMax;
    Vector handRestPos;
    
    double velocity;
    double maxVel;
    int    period;
    
    Vector controlledJoints;
     
    /* class variables */

    PolyDriver robotDevice;                        // joints torques input port
    RpcServer setPort;                             // input/output port that receives commands from user (typically to change some parameters), and sends replies
    BufferedPort< Bottle > inputPort;              // input port that receives hand joints values 

    /* pointer to a new thread to be created and started in configure() and stopped in close() */ 

    HandControlThread    *handControlThread;
    UserCmdThread        *cmdT;

public:
   
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    double getPeriod(); 
    bool updateModule();
};



#endif


