// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


#ifndef __ICUB_HANDTACTILECONTROL_MODULE_H__
#define __ICUB_HANDTACTILECONTROL_MODULE_H__ 

#include <iostream>
#include <string>
#include <vector>

#include <math.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
 
using namespace std;

using namespace yarp;
using namespace yarp::os; 
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::sig;

using namespace iCub::iKin;


class HandTactileControlThread : public RateThread
{
   /* class variables */ 

    IPositionControl2 *pos;
    IPositionDirect *posDir;
    IControlMode2 *ctrlMode;
    IVelocityControl *vel;
    IEncoders *encs;
    IControlLimits *limits;
    ICartesianControl *cartCtrl;
    
    string partName;
    string handName;
    
    Vector encoders;
    Vector v_estimate;
    Vector p_command;
    Vector v_command;
    Vector def_hand_p;
    Vector def_hand_o;
    Vector target_hand_p;
    Vector target_hand_o;
    Vector ctrl_param;
    
    Matrix fingersSensitivityScale;
    Matrix fingerTaxelsData;
    Matrix fingerTaxelsDataBinary;
    Matrix fingerTaxelsDataContacts;
    Matrix fingertipForcesLocal;
    Matrix fingertipForcesGlobal;
   
    // Additional Matrix to save the positions of each finger 
    Matrix fingertipPose;

    Matrix handJointsPos;
    
    int    nAxes;
    int    ctrlJoints;
    int    nParameters;
    
    int    ctrlStep;
    int    periodInMsec;
    double periodInSec;
    
    bool   motionStarted;
    bool   motionDone;
 
    
    bool readInputPort();
    void readData();
    void readEncs(bool v=true);
    void dumpData();
    void sendData();
    void goHomeArm();
    void setArm(Vector armConf);
    void setDefHandPoseToThis();
    void handMoveToPose(Vector xd, Vector od);
    void updateRef(); 
    void closeHandToContact();
    void checkMotionDone_posDir_handGrasp(bool &done);
    void computeGraspMetric();
    void openHand();
    bool readFingerSkinCompData(bool block=false);
    void updateFingertipForces();
    void sumUnitVectors(std::vector<double>& A, std::vector<double>& B, std::vector<double>& res);
    void getUnitVector(int index,std::vector<double>& unitVector);
    void checkFingersContacts(bool *contactsList);
    void stopFinger(int index);
    void squeezingStep();  
    
    /* thread parameters: they are pointers so that they refer to the original variables in myModule */

    PolyDriver             *robotDevice;
    PolyDriver             *cartDevice;
    
    iCubFinger   *fingers;                         // to get access to individual fingers
    
    BufferedPort< Bottle > *inputPort; //target hand configuration (i.e. hand position wrt object)
    BufferedPort< Vector > *portSkinCompIn; //skin data (compensated)
    BufferedPort< Bottle > *outputPort; //grasp metric computed on the current grasp (based on the fingetip contacts and estimated interaction forces)
    
    Vector *jointLimitsMin;
    Vector *jointLimitsMax;
    Vector *armRestPos;
    
    Vector *controlledJointsVector; 
    
    double *handPosRef;
    double *handVelRef;
    double *armFullConf;
    int *controlledJoints;
    int *controlModesHand;
    int *controlModesArm;
    
    double velocity;
    double refSpeed;
    double startT;
    double moveT;
    double cycleTime;
    double maxV;
    
public: 
  
    int controlMode; // 0 - idle; 1 - position direct; 2 - postion control with bell-shaped trajectories; 3 - contact sensing test

    /* class methods */
    HandTactileControlThread(PolyDriver *robDev, PolyDriver *cartDev, string pName, Vector *jLimitsMin, Vector *jLimitsMax, Vector *hRestPos, double vel, double mV, int per, BufferedPort< Bottle > *inPort, BufferedPort< Vector > *skinCompPort, BufferedPort< Bottle > *outPort, Vector *cJoints);
    bool threadInit();     
    void threadRelease();
    void run(); 
    void simControlStep();
};

class UserCmdThread : public Thread
{
    HandTactileControlThread *handConThread;
    RpcServer *setPort;                        // input/output port that receives commands from user (typically to change some parameters), and sends replies
    
public:
    
    UserCmdThread(HandTactileControlThread *hcThread, RpcServer *sPort );
    bool threadInit();     
    void threadRelease();
    void run(); 
    void simControlStep();
    
    void checkUserCmd();
  
};


class HandTactileControlModule : public RFModule
{
    /* module parameters */

    string moduleName;
    string robotName;
    string partName;

    string remotePortName;  
    string localPortName;  
    
    string setPortName;
    string inputPortName;
    string skinCompPortName;
    string outputPortName;
    
    string deviceName;
    
    Vector jointLimitsMin;
    Vector jointLimitsMax;
    Vector armRestPos;
    
    double velocity;
    double maxVel;
    int    period;
    
    Vector controlledJoints;
     
    /* class variables */

    PolyDriver robotDevice;                        // to control robot joints
    PolyDriver cartDevice;                         // to control the robot in Cartesian space
    RpcServer setPort;                             // input/output port that receives commands from user (typically to change some parameters), and sends replies
    BufferedPort< Bottle > inputPort;              // input port that receives target hand configuration values 
    BufferedPort< Vector > skinCompPort;         // port to read skin data (compensated)
    BufferedPort< Bottle > outputPort;              // output port that sends grasp metric values

    /* pointer to a new thread to be created and started in configure() and stopped in close() */ 

    HandTactileControlThread    *handControlThread;
    UserCmdThread        *cmdT;

public:
   
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    double getPeriod(); 
    bool updateModule();
};



#endif


