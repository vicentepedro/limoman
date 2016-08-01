
#include "../include/handControl.h"


const int DEFAULT_CTRL_JOINTS           = 9;
const int DEFAULT_GLOVE_DATA_DIM        = 14;    

const double MAX_VEL_DEF = 40.0;

//const double HAND_DEF_POS[]={40.0,  20.0,20.0,20.0,  20.0,20.0, 20.0,20.0,  20.0}; 
const double HAND_DEF_VEL = 20.0;

const int DISPLAY_RATE = 50; //display information on screen every 50 control steps
const double INIT_WAIT_TIME = 2.0; 

const int DEFAULT_CONTROL_PERIOD = 10;

const bool ICUB_SIM = false;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */
bool HandControlModule::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */
    cout << "Processing program parameters\n";
    

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                           Value("handControl"), 
                           "module name (string)").asString();

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());

   /* now, get the rest of the parameters */

   /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */

    robotName               = rf.check("robot", 
                              Value("icubSim"), 
                              "Robot name (string)").asString();

    partName                = rf.check("part",
                              Value("right_arm"),
                              "Robot limb name, e.g., right_arm (string)").asString();
   
    remotePortName          = "/" + robotName + "/" + partName;
    
    localPortName           = "/" ;
    localPortName           += getName( rf.check("localPort", 
                               Value("/local"),
                               "Local port for device connection").asString()
                               );                       
    inputPortName          = "/" ;
    inputPortName          += getName( rf.check("inputPort", 
                               Value("/data:i"),
                               "Input port that receives hand joints values ").asString()
                               );
    
    setPortName          = "/" ;
    setPortName          += getName( rf.check("setPort", 
                               Value("/set:rpc"),
                               "Input/Output port to set parameters").asString()
                               );

    deviceName              = rf.check("deviceName", 
                              Value("remote_controlboard"), 
                              "Device to open (string)").asString();

    Bottle activeJointsBottle;
    if( rf.check("activeJoints", "Active joints for the given robot part, e.g., 1 1 0 1 1") )
        activeJointsBottle = rf.findGroup("activeJoints").tail();
        
    if( activeJointsBottle.size() == 0 )
    {
        cout << getName() << ": you must provide a valid active joints list for the given robot part, e.g., 1 1 0 1 1" << endl;
        return false;
    }

    Bottle jointsMaxBottle, jointsMinBottle, restPosBottle;
    if( rf.check("jointsMin", "Admissable minimum values for joints positions") )
        jointsMinBottle = rf.findGroup("jointsMin").tail();
    if( rf.check("jointsMax", "Admissable maximum values for joints positions") )
        jointsMaxBottle = rf.findGroup("jointsMax").tail();
    
    if( rf.check("restPos", "Rest position of the head") )
        restPosBottle = rf.findGroup("restPos").tail();

    velocity                = rf.check("velocity", 
                              Value(HAND_DEF_VEL), 
                              "Speed of position movement").asDouble();
			      
    maxVel                  = rf.check("maxVel", 
                              Value(MAX_VEL_DEF), 
                              "Maximum speed for FB control").asDouble();
                              
    period                  = rf.check("period", 
                              Value(DEFAULT_CONTROL_PERIOD), 
                              "Thread period").asInt();

    /* do all initialization here */
    
    /* open ports  */ 
    
    if (!inputPort.open(inputPortName.c_str())) 
    {
        cout << getName() << ": unable to open port " << inputPortName.c_str() << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!setPort.open(setPortName.c_str())) 
    {
        cout << getName() << ": unable to open port " << setPortName.c_str() << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    
    /* connect to remote device  */
    Property options;
    options.put("device", deviceName.c_str());             // device to open
    options.put("local", localPortName.c_str());           // local port name
    options.put("remote", remotePortName.c_str());         // where we connect to
    
   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
  
    // Create a device
    robotDevice.open(options);
    if (!robotDevice.isValid()) 
    {
        cout << getName() << ": unable to connect to device " << deviceName << endl;
        return false;
    }

    // Check if control, encoders and limits interfaces are OK    
    IPositionControl2 *pos;
    IPositionDirect  *posD;
    IVelocityControl *vel;
    IEncoders *encs;
    IControlLimits *lim;
    IControlMode2 *ctrlM;
    if (!robotDevice.view(pos) || !robotDevice.view(posD) || !robotDevice.view(encs) || !robotDevice.view(vel) || !robotDevice.view(lim) || !robotDevice.view(ctrlM)) 
    {
        cout << getName() << ": problems acquiring interfaces to " << deviceName << endl;
        return false;
    }
    cout << "Acquired temporary robot interface.\n";

    // Fill the vectors with current active joints limits
    int nAxes;
    int cAxes=0;
    pos->getAxes(&nAxes);
    
    jointLimitsMin.resize(nAxes,0.0);
    jointLimitsMax.resize(nAxes,0.0);
    handRestPos.resize(nAxes,0.0);
    for(int i = 0; (i < nAxes) && (i < activeJointsBottle.size()); i++)
        if( activeJointsBottle.get(i).asInt() == 1 )
        {
	    cAxes++;
            lim->getLimits(i, &jointLimitsMin[i], &jointLimitsMax[i]);
            if( i < jointsMinBottle.size() && jointsMinBottle.get(i).asDouble() > jointLimitsMin[i] && jointsMinBottle.get(i).asDouble() < jointLimitsMax[i])
                jointLimitsMin[i] = jointsMinBottle.get(i).asDouble();
            if( i < jointsMaxBottle.size() && jointsMaxBottle.get(i).asDouble() > jointLimitsMin[i] && jointsMaxBottle.get(i).asDouble() < jointLimitsMax[i])
                jointLimitsMax[i] = jointsMaxBottle.get(i).asDouble();
        }
        
    controlledJoints.resize(cAxes);
    
    cout << "\n" << partName.c_str() << " of " << robotName.c_str() << " has " << nAxes << " controllable axis, but only " << cAxes << " will be controlled. \n"; 
    
    int j=0;
    for (int i= 0; i < nAxes; i++)
    {
        if( activeJointsBottle.get(i).asInt() == 1 )
	{
	    controlledJoints[j]=i;
	    j++;
	}
    }
    
    if (j==cAxes)
    {
        cout << "This is the list of controlled joints: " << endl;
	for (int i= 0; i < cAxes; i++)
	{
	    cout << (int)controlledJoints[i] << " ";
	}
        cout << endl;
    }
    else
    {
        cout << "\n [WARNING] - Problem when creating the list of controlled joints.\n";
    }
        
    cout << "Using the following joint limits:\n";
    for(int i = 0; (i < nAxes) ; i++)
        cout << jointLimitsMin[i] << "\t";
    cout << endl;
    for(int i = 0; (i < nAxes) ; i++)
        cout << jointLimitsMax[i] << "\t";
    cout << endl;
    
    for (int i= 0; i < nAxes; i++)
    {
        handRestPos[i]=restPosBottle.get(i).asDouble();
    }
    
    
        
    /* create the thread and pass pointers to the module parameters */

    handControlThread = new HandControlThread(&robotDevice, &jointLimitsMin, &jointLimitsMax, &handRestPos, velocity, maxVel, period, &inputPort, &controlledJoints);

    /* now start the thread to do the work */

    handControlThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    
    cmdT = new UserCmdThread(handControlThread,&setPort);
    cmdT->start();

    return true ;      // let the RFModule know everything went well
                       // so that it will then run the module
}

bool HandControlModule::interruptModule()
{
    inputPort.interrupt();

    return true;
}


bool HandControlModule::close()
{
    inputPort.close();
    robotDevice.close();

    /* stop the thread */
    handControlThread->stop();
    cmdT->stop();

    delete handControlThread;
    delete cmdT;

    return true;
}

/* Called periodically every getPeriod() seconds */ 

bool HandControlModule::updateModule()
{
   return true;
} 



double HandControlModule::getPeriod()
{
   /* module periodicity (seconds), called implicitly by myModule */
   return 0.5;
}

//**********************************
  
UserCmdThread::UserCmdThread(HandControlThread *hcThread, RpcServer *sPort )
{
    handConThread=hcThread;
    setPort=sPort; 

}

bool UserCmdThread::threadInit()
{
    return true;
}

void UserCmdThread::checkUserCmd()
{
    Bottle userCmd, rep;
    string msg = "";
    string tmpSa, tmpSb;
    int tmpI;
    userCmd.clear();
    if ( setPort->read(userCmd,true) )
    {
        
	fprintf(stderr,"Got user message: %s\n", userCmd.toString().c_str());
	
	rep.clear();
	//rep.addString("ack");
	//setPort->reply(rep);
	
	tmpSa = userCmd.get(0).asString();
	
	if (tmpSa=="set")
	{
	    tmpSb = userCmd.get(1).asString();
	    if (tmpSb =="mode")
	    {
		tmpI = userCmd.get(2).asInt();
		fprintf(stderr,"Desired control mode is: %d\n", tmpI);
	        fprintf(stderr,"Current control mode is: %d\n", handConThread->controlMode);
	        handConThread->controlMode = tmpI;
	        fprintf(stderr,"New control mode is: %d\n", handConThread->controlMode);
		rep.addString("done");
		rep.addInt(handConThread->controlMode);
	        setPort->reply(rep);
	    }
	    else
	    {
		fprintf(stderr,"\n--ERROR: unidentified message--\n");
		rep.addString("fail");
	        setPort->reply(rep);
	    }
	    
	}
	else if (tmpSa=="get")
	{
	    tmpSb = userCmd.get(1).asString();
	    if (tmpSb =="mode")
	    {
	        rep.addString("done");
		rep.addInt(handConThread->controlMode);
	        setPort->reply(rep);
		fprintf(stderr,"Control mode sent is: %d\n", handConThread->controlMode);
	    }
	    else
	    {
		fprintf(stderr,"\n--ERROR: unidentified message--\n");
		rep.addString("fail");
	        setPort->reply(rep);
	    }
	  
	}
	else
	{
	    fprintf(stderr,"\n--ERROR: unidentified message--\n");
	    rep.addString("fail");
	    setPort->reply(rep);
	}
	
    }
  
}

void UserCmdThread::run()
{
  
    while ( !isStopping() )
    {
        checkUserCmd(); //wait for a user commands to set some parameters of DynamicControl Thread
    }

}

void UserCmdThread::threadRelease()
{
    

}

  
//*********************************** 


HandControlThread::HandControlThread(PolyDriver *robDev, Vector *jLimitsMin, Vector *jLimitsMax, Vector *hRestPos, double vel, double mV, int per, BufferedPort< Bottle > *iPort, Vector *cJoints) : RateThread(per)
{
    robotDevice  = robDev;
    inputPort   = iPort;
    jointLimitsMin = jLimitsMin;
    jointLimitsMax = jLimitsMax;
    handRestPos     = hRestPos;
    velocity = vel;
    periodInMsec = per;
    periodInSec = (double)per/1000.0;
    maxV = mV;
    
    refSpeed = 0.0;
    
    ctrlStep=0;
    
    controlledJointsVector = cJoints;
    gloveDataDim = DEFAULT_GLOVE_DATA_DIM;
    
    controlMode=0;   // 0 - idle; 1 - position direct; 2 - postion control with bell-shaped trajectories
    
    motionStarted=false;
    motionDone=false;
    ctrlJoints=controlledJointsVector->size();
    
} 

bool HandControlThread::threadInit() 
{
    // Control and encoders interface    
    if (!robotDevice->view(pos) || !robotDevice->view(posDir) || !robotDevice->view(encs) || !robotDevice->view(vel) || !robotDevice->view(limits) || !robotDevice->view(ctrlMode)) 
    {
        cout << "Problems acquiring interfaces to device driver." << endl;
        return false;
    }
    cout << "Acquired robot interface.\n";

    pos->getAxes(&nAxes);
    encoders.resize(nAxes);
    v_estimate.resize(nAxes);
    p_command.resize(nAxes);
    v_command.resize(nAxes);
    
    posRef.resize(ctrlJoints,1);
    gloveData.resize(gloveDataDim,1);
    handJointsPos.resize(ctrlJoints,1);    
    
    //mapping from glove angle data (human hand) to iCub hand motors
    jointsMap.resize(ctrlJoints,gloveDataDim);
    
    //to iCub hand fingers abduction
    jointsMap(0,11)=0.33;  //index-middle abduction
    jointsMap(0,12)=0.33;  //middle-ring abduction
    jointsMap(0,13)=0.33;  //ring-pinky abduction

    //to iCub hand thumb opposition
    jointsMap(1,0)=1.0;   //thumb opposition/abduction
    
    //to iCub hand thumb proximal
    jointsMap(2,1)=1.0;   //thumb proximal flexion
    //jointsMap(2,2)=1.0;   //thumb proximal flexion /***********MODIFIED BECAUSE OF BROKEN JOINT **********/
    
    //to iCub hand thumb distal
    jointsMap(3,2)=1.0;   //thumb distal flexion 
    //jointsMap(3,1)=1.0;   //thumb distal flexion /***********MODIFIED BECAUSE OF BROKEN JOINT **********/
    
    //to iCub hand index proximal
    jointsMap(4,3)=1.0;   //index proximal flexion 
    
    //to iCub hand index distal
    jointsMap(5,4)=1.0;   //index distal flexion
    
    //to iCub hand middle proximal
    jointsMap(6,5)=1.0;   //middle proximal flexion
    
    //to iCub hand middle distal
    jointsMap(7,6)=1.0;   //middle distal flexion
    
    //to iCub hand ring+pinky flexion
    jointsMap(8,7)=0.3;  //ring proximal flexion
    jointsMap(8,8)=0.2;  //ring distal flexion
    jointsMap(8,9)=0.3;  //pinky proximal flexion
    jointsMap(8,10)=0.2; //pinky distal flexion
    
    //gains from glove angle data (human hand) to iCub hand motors
    jointsGain.resize(ctrlJoints,ctrlJoints);
    
    //to iCub hand fingers abduction
    jointsGain(0,0)=0.50; 

    //to iCub hand thumb opposition
    jointsGain(1,1)=0.70; //0.70 (default) 0.85 (max?)
    
    //to iCub hand thumb proximal
    jointsGain(2,2)=2.25;  //2.25 (default), 2.5 (max?)
    //jointsGain(2,2)=1.0;  //2.5   /***********MODIFIED BECAUSE OF BROKEN JOINT **********/
    
    //to iCub hand thumb distal
    jointsGain(3,3)=1.5;  //2.0
    //jointsGain(3,3)=0.0;  //2.0   /***********MODIFIED BECAUSE OF BROKEN JOINT **********/
    
    //to iCub hand index proximal
    jointsGain(4,4)=1.0;  //1.0
    
    //to iCub hand index distal
    jointsGain(5,5)=1.5;  //2.0  
    
    //to iCub hand middle proximal
    jointsGain(6,6)=0.8;  //1.0
    
    //to iCub hand middle distal
    jointsGain(7,7)=1.25;  //1.5 (2.0 on sim...)  
    
    //to iCub hand ring+pinky flexion
    jointsGain(8,8)=2.25;  //2.25
    
    controlledJoints = new int[ctrlJoints];
    controlModes = new int[ctrlJoints];
    handPosRef = new double[ctrlJoints];
    handVelRef = new double[ctrlJoints];
    
    for (int i= 0; i < ctrlJoints; i++)
    {
	controlledJoints[i] = (*controlledJointsVector)[i];
    }
    
    for (int i= 0; i < ctrlJoints; i++)
    {
	handVelRef[i] = velocity;
    }
    
    pos->setRefSpeeds(ctrlJoints,controlledJoints,handVelRef);
    
    int j;
    
    cout << "This is the list of controlled joints. " << endl;
    
    cout << "JOINTS: ";
    for (int i= 0; i < ctrlJoints; i++)
    {
	cout << controlledJoints[i] << " ";
    }
    cout << endl;
    
    j=0;
    cout << "MIN POS: ";
    for (int i= 0; i < nAxes; i++)
    {
        if (controlledJoints[j]==i)
	{
	    j++; 
	    cout << (*jointLimitsMin)[i] << " ";
	}
    }
    cout << endl;
    
    j=0;
    cout << "MAX POS: ";
    for (int i= 0; i < nAxes; i++)
    {
        if (controlledJoints[j]==i)
	{
	    j++; 
	    cout << (*jointLimitsMax)[i] << " ";
	}
    }
    cout << endl;
    
    cout << "MAX VEL: ";
    for (int i= 0; i < ctrlJoints; i++)
    {
	cout << maxV << " ";
    }
    cout << endl;
    
    j=0;
    cout << "DEFAULT VEL: ";
    for (int i= 0; i < nAxes; i++)
    {
        if (controlledJoints[j]==i)
	{
	    j++; 
	    pos->getRefSpeed(i,&refSpeed);
	    cout << refSpeed << " ";
	}
    }
    cout << endl;

    fprintf(stderr,"\n\nHandControlThread -- Control started!\n\n");
    
    startT=Time::now();

    return true;
}


void HandControlThread::updateRef()
{
  
    double tmpD;

    for (int i = 0; i < ctrlJoints; i++) 
    {
        handPosRef[i]=posRef(i,0);
    }

    handPosRef[0]=(*jointLimitsMax)[7] - handPosRef[0]; //fingers abduction works in the opposite direction...
    
    
    if (ICUB_SIM)
    {
        tmpD = handPosRef[1];
        handPosRef[1] = handPosRef[2];
        handPosRef[2] = tmpD;
	    handPosRef[2]=(*jointLimitsMax)[9] - handPosRef[2];
    }
    
}

void HandControlThread::handPosMove()   
{
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION);
	}
    }
	    
    pos->positionMove(ctrlJoints, controlledJoints, handPosRef);  // position move command using bell-shaped velocities, non-blocking
    
}

void HandControlThread::handPosMoveDirect()   
{
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION_DIRECT)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION_DIRECT);
	}
    }
    
    posDir->setPositions(ctrlJoints, controlledJoints, handPosRef);  // position move command, non-blocking
    
}

void HandControlThread::readEncs(bool v)
{
  
    if (!encs->getEncoders(encoders.data()))
    {
        if (v)
	{  
            fprintf(stderr,"\n\n WARNING! -- Problem reading joints positions \n\n");
	}
    }
    
    int j=0;
    
    for (int i=0; i < nAxes; i++)
    {
        if(controlledJoints[j]==i)
	{
	    handJointsPos(j,0) = encoders[i];
	    j++;
	}
    }
    
    if (j!=ctrlJoints)
    {
        cout << "\n[WARNING] - Problem in assigning encoders values...\n";
    }
    
}

bool HandControlThread::readData()
{
    
    if(Bottle *bot=inputPort->read(false))  // to take target reference: visibleFlag(0/1), uL, vL, uR, vR
    {   
        if (bot->size() != gloveDataDim)
	{
            cout << "\n[WARNING] - Problem in size of packet received from data glove...\n";
	    cout << bot->size() << " values received, " << gloveDataDim << " expected.\n";
        }
    
        for (int i=0; i<gloveDataDim; i++)
	{
	    gloveData(i,0)=bot->get(i).asDouble(); 
	}
	
	//mapping to robot joints
	posRef = jointsGain * jointsMap * gloveData;
	
	return true;
    }

    return false;
  
}


void HandControlThread::dumpData()
{
    /*
    Bottle &db = dumpPort->prepare();
    db.clear();
    
    for(int i = 0; i < nAxes; i++)
    {
        db.addDouble(encoders[i]);
    }
    
    for(int i = 0; i < imgCoordDim; i++)
    {
        db.addDouble(visPos[i]);
    }
    
    dumpPort->write(); 
    */
  
}


void HandControlThread::run()
{
    ctrlStep++;  
  
    if (ctrlStep%DISPLAY_RATE==0)
    {  
        fprintf(stderr,"\n\nCycle time (real control loop time): \n");
	fprintf(stderr,"%.4lf\n\n",Time::now()-cycleTime);
    }
    
    cycleTime=Time::now();
    
    readEncs();
    readData();
    updateRef();
    
    switch (controlMode)
    {
      
    case 0:
       //pos->stop();
       fprintf(stderr, "\n\n-- no hand control (idle)--\n\n");
       Time::delay(0.5);
       break;
	
    case 1:
       handPosMoveDirect();
       break;
	
    case 2: 
       handPosMove();
       break;
       
    default:
       pos->stop();
       fprintf(stderr, "\n\n-- [WARNING] - No control mode has been specified for hand control --\n\n");
       Time::delay(0.1);
       break;
       
    }
       
    if (ctrlStep%DISPLAY_RATE==0)
    {  
        fprintf(stderr,"\n\nCycle computation time: \n");
	fprintf(stderr,"%.4lf\n\n",Time::now()-cycleTime);
    }
    
    if (ctrlStep%DISPLAY_RATE==0)
    {
        cout << "\nControlled joints: \n";
        for (int i = 0; i < ctrlJoints; i++) 
        {
            cout << controlledJoints[i] << " ";
        }
        cout << "\n";
        cout << "\nControl references: \n";
        for (int i = 0; i < ctrlJoints; i++) 
        {
            cout << handPosRef[i] << " ";
        }
    }
    
}


void HandControlThread::threadRelease() 
{    
    delete[] controlledJoints;
    delete[] controlModes;
    delete[] handPosRef;
}


