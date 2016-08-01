
#include "../include/handTactileControl.h"


const int DEFAULT_CTRL_JOINTS           = 9;  // For the fingers. Maximum is 9.
const int DEFAULT_N_PARAMETERS          = 2;  // Control parameters for the hand. Typically, y,z (or x,y,z) Cartesian position.  

const int N_FINGERS  = 5;
const int N_TAXELS   = 12;

const double MAX_VEL_DEF = 40.0;

const double TOUCH_THR = 0.5;

/*************   FOR SIMULATOR  ********/

const double HAND_DEF_HOME[]={20.0,       80.0, 10.0, 10.0,      10.0, 10.0,     10.0, 10.0,      10.0}; 
const double HAND_DEF_TARGET[]={20.0,     80.0, 25.0, 50.0,      25.0, 70.0,     25.0, 70.0,     120.0}; 
/*************************/

/*************   FOR REAL ROBOT********
const double HAND_DEF_HOME[]={40.0,  20.0, 10.0,10.0,  10.0,10.0, 10.0,10.0,  10.0}; 
const double HAND_DEF_TARGET[]={40.0,     70.0, 10.0, 40.0,     60.0, 50.0,    70.0, 60.0,    140.0};
/**********************/

const double HAND_VEL_COEFF = 3.0;
const double HAND_DEF_VEL = 50.0;
const double MAX_GRASP_DURATION = 10.0;
const double GRASP_POS_THR = 1.0;
const double HAND_SQUEEZE_STEP[]={0.0,  0.0, 3.0, 5.0,  3.0,5.0,  3.0,5.0,  10.0};

const double ARM_DEF_HOME[] = {-50.0,  60.0,  0.0,  40.0, 0.0,  0.0,   0.0,  40.0,  20.0,20.0,20.0,  20.0,20.0, 20.0,20.0,  20.0};

const int DISPLAY_RATE = 50; //display information on screen every 50 control steps
const double INIT_WAIT_TIME = 2.0; 


const int DEFAULT_CONTROL_PERIOD = 10;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */
bool HandTactileControlModule::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */
    cout << "Processing program parameters\n";
    

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                           Value("handTactileControl"), 
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
                               "Input port that receives target hand configuration values").asString()
                               );
    
    skinCompPortName          = "/" ;
    skinCompPortName          += getName( rf.check("skinCompPort", 
                               Value("/skinComp:i"),
                               "Input port that receives compensated skin data").asString()
                               );
    
    outputPortName          = "/" ;
    outputPortName          += getName( rf.check("outputPort", 
                               Value("/data:o"),
                               "Output port that sends grasp metric values").asString()
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

    Bottle jointsMaxBottle, jointsMinBottle, restPosBottleArm;
    if( rf.check("jointsMin", "Admissable minimum values for joints positions") )
        jointsMinBottle = rf.findGroup("jointsMin").tail();
    if( rf.check("jointsMax", "Admissable maximum values for joints positions") )
        jointsMaxBottle = rf.findGroup("jointsMax").tail();
    
    if( rf.check("restPosArm", "Rest position of the arm (including the hand)") )
        restPosBottleArm = rf.findGroup("restPosArm").tail();

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

    
    if (!skinCompPort.open(skinCompPortName.c_str())) 
    {
        cout << getName() << ": unable to open port " << skinCompPortName.c_str() << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!outputPort.open(outputPortName.c_str())) 
    {
        cout << getName() << ": unable to open port " << outputPortName.c_str() << endl;
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
    
    
    /****************   CARTESIAN CONTROL**********/
    ICartesianControl *cCtrl;
    
    remotePortName          = "/" + robotName + "/cartesianController/" + partName;
    
    // open the cartesian client
    Property optCart("(device cartesiancontrollerclient)");
    optCart.put("remote",remotePortName.c_str());
    optCart.put("local",(localPortName+"/"+partName+"/cartesian").c_str());
    if (!cartDevice.open(optCart))
    {
        cout << getName() << ": problems acquiring Cartesian interface" << endl;
        return false;
    }

    // open views
    cartDevice.view(cCtrl);
    
    /***************************/
    
    
    
    /*******************************/
    // Fill the vectors with current active joints limits
    int nAxes;
    int cAxes=0;
    pos->getAxes(&nAxes);
    
    jointLimitsMin.resize(nAxes,0.0);
    jointLimitsMax.resize(nAxes,0.0);
    
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
        cout << "This is the list of controlled hand joints: " << endl;
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

    armRestPos.resize(nAxes);    
    for (int i= 0; i < nAxes; i++)
    {
        armRestPos[i]=restPosBottleArm.get(i).asDouble();
    }
    
    
     
    /* create the thread and pass pointers to the module parameters */
    
    fprintf(stderr,"Now open the hand tactile control thread...\n");
       
    handControlThread = new HandTactileControlThread(&robotDevice, &cartDevice, partName, &jointLimitsMin, &jointLimitsMax, &armRestPos, velocity, maxVel, period, &inputPort, &skinCompPort, &outputPort, &controlledJoints);

    fprintf(stderr,"...done.\n");
    
    /* now start the thread to do the work */

    fprintf(stderr,"Now start the hand tactile control thread...\n");
    
    handControlThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    
    fprintf(stderr,"...done.\n");
    
    cmdT = new UserCmdThread(handControlThread,&setPort);
    cmdT->start();

    return true;       // let the RFModule know everything went well
                       // so that it will then run the module
}

bool HandTactileControlModule::interruptModule()
{
    inputPort.interrupt();

    return true;
}


bool HandTactileControlModule::close()
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

bool HandTactileControlModule::updateModule()
{
   return true;
} 



double HandTactileControlModule::getPeriod()
{
   /* module periodicity (seconds), called implicitly by myModule */
   return 0.5;
}

//**********************************
  
UserCmdThread::UserCmdThread(HandTactileControlThread *hcThread, RpcServer *sPort )
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


HandTactileControlThread::HandTactileControlThread(PolyDriver *robDev, PolyDriver *cartDev, string pName, Vector *jLimitsMin, Vector *jLimitsMax, Vector *aRestPos, double vel, double mV, int per, BufferedPort< Bottle > *iPort, BufferedPort< Vector > *sPort, BufferedPort< Bottle > *oPort, Vector *cJoints) : RateThread(per)
{
    robotDevice  = robDev;
    cartDevice  = cartDev;
    partName    = pName;
    inputPort   = iPort;
    portSkinCompIn = sPort;
    outputPort   = oPort;
    jointLimitsMin = jLimitsMin;
    jointLimitsMax = jLimitsMax;
    armRestPos     = aRestPos;
    velocity = vel;
    periodInMsec = per;
    periodInSec = (double)per/1000.0;
    maxV = mV;
    
    refSpeed = 0.0;
    
    ctrlStep=0;
    
    controlledJointsVector = cJoints;
    nParameters = DEFAULT_N_PARAMETERS;
    
    controlMode=0;   // 0 - idle; 1 - wait for next target; 2 - move to target
    
    motionStarted=false;
    motionDone=false;
    ctrlJoints=controlledJointsVector->size();
    
} 


bool HandTactileControlThread::threadInit() 
{
    // Control and encoders interface    
    if (!robotDevice->view(pos) || !robotDevice->view(posDir) || !robotDevice->view(encs) || !robotDevice->view(vel) || !robotDevice->view(limits) || !robotDevice->view(ctrlMode)) 
    {
        cout << "Problems acquiring interfaces to joints-level control device drivers." << endl;
        return false;
    }
    cout << "Acquired robot interface (joints-level control).\n";
    
    if (!cartDevice->view(cartCtrl))
    {
        cout << "Problems acquiring Cartesian interface" << endl;
        return false;
    }
    cout << "Acquired Cartesian interface.\n";

    pos->getAxes(&nAxes);
    encoders.resize(nAxes);
    v_estimate.resize(nAxes);
    p_command.resize(nAxes);
    v_command.resize(nAxes);
    
    def_hand_p.resize(3);
    def_hand_o.resize(4);
    target_hand_p.resize(3);
    target_hand_o.resize(4);
    ctrl_param.resize(nParameters);
    
    /*************  FINGERS CONTROL ******************/
    
    if (partName=="right_arm")
    {
        handName = "right";
    }
    else if (partName=="left_arm")
    {
        handName = "left";
    } 
    else 
    {
        handName = "none";
    } 
    
    fingers = new iCubFinger[5];  //Vector of 5 iCubFingers: 0="thumb", 1="index", 2="middle", 3="ring", 4="little"
    
    iCubFinger th(handName+"_thumb");
    iCubFinger in(handName+"_index");
    iCubFinger mi(handName+"_middle");
    iCubFinger ri(handName+"_ring");
    iCubFinger li(handName+"_little");
    
    if (handName!="none")
    {
	fingers[0]=th;
	fingers[1]=in;
	fingers[2]=mi;
	fingers[3]=ri;
        fingers[4]=li;
    }
    else
    {
        cout << "WARNING: Part is not an arm and/or hand name is not correctly specified" << endl;
	return false;
    }
    
    cout << "Controlling the following fingers:" << endl;
    for (int i = 0; i < N_FINGERS; i++)   // compute unitary forces applied on the fingertips, in the fingertip reference frame
    {
	fprintf(stderr,"\n%s\n",fingers[i].getType().c_str()); 
    }
    
    /*******************************/
    
    fingersSensitivityScale.resize(N_FINGERS,1); //typically: 5 fingertips
    for (int i= 0; i < N_FINGERS; i++)
    {
	fingersSensitivityScale(i,0) = 1.0; //default
    }
    
    fingerTaxelsData.resize(N_FINGERS,N_TAXELS); //typically: 5 fingertips, 12 taxels per fingertip
    fingerTaxelsDataBinary.resize(N_FINGERS,N_TAXELS); //typically: 5 fingertips, 12 taxels per fingertip
    cout << "\nRead tactile data...\n";
    if (!readFingerSkinCompData(true))
    {
        cout << "WARNING - Problem reading tactile sensors.\n";
    }
    cout << "...done.\n\n";
    
    fingertipForcesLocal.resize(N_FINGERS,3);
    fingertipForcesGlobal.resize(N_FINGERS,3);
     
    handJointsPos.resize(ctrlJoints,1);    
    
    controlledJoints = new int[ctrlJoints];
    controlModes = new int[ctrlJoints];
    handPosRef = new double[ctrlJoints];
    handVelRef = new double[ctrlJoints];
    armFullConf = new double[nAxes];
    
    for (int i= 0; i < ctrlJoints; i++)
    {
	controlledJoints[i] = (*controlledJointsVector)[i];
    }
    
    for (int i= 0; i < ctrlJoints; i++)
    {
	//handVelRef[i] = velocity;
          handVelRef[i] = (HAND_DEF_TARGET[i] - HAND_DEF_HOME[i]) / HAND_VEL_COEFF;
	  if (handVelRef[i] < 2.0)
	  {
	      handVelRef[i] = 2.0;
	  }
    }
    
    for (int i= 0; i < nAxes; i++)
    {
	//armFullConf[i] = ARM_DEF_HOME[i];
	armFullConf[i] = (*armRestPos)[i];
    }
    
    pos->setRefSpeeds(ctrlJoints,controlledJoints,handVelRef);
    //posDir->setRefSpeeds(ctrlJoints,controlledJoints,handVelRef);   //how to set joints velocities with position direct???
    
    int j;
    
    cout << "This is the list of controlled finger joints. " << endl;
    
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
    
    fprintf(stderr,"\n\nHandControlThread -- Going home\n\n");
    
    cout << "GOING TO ARM CONFIGURATION: " << endl;
    for (int i= 0; i < nAxes; i++)
    {        
	cout << (*armRestPos)[i] << " ";
    }
    cout << endl;
    
    goHomeArm();
    Time::delay(2.0);

    fprintf(stderr,"\n\nHandControlThread -- Control started!\n\n");
    
    startT=Time::now();

    return true;
}

void HandTactileControlThread::goHomeArm()
{
    ctrlMode->getControlModes(controlModes);
    for(int i=0; i<nAxes; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION)
	{
	    ctrlMode->setControlMode(i,VOCAB_CM_POSITION);
	}
    }
    
    for (int i= 0; i < nAxes; i++)
    {
	armFullConf[i] = (*armRestPos)[i];
    }
	    
    pos->positionMove(armFullConf);  // position move command using bell-shaped velocities, non-blocking
    
    /********** COMMENT THIS TO MAKE IT NON-BLOCKING ***********/
    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();
    while(!done && elapsedTime<5.0)
    {
        pos->checkMotionDone(&done);
	Time::delay(0.04);
	elapsedTime= Time::now()-startTime;
    }
    /***********************************************************/
    
    pos->stop();
    Time::delay(0.5);
    
    Vector x0,o0;
    cartCtrl->getPose(x0,o0);
    
    def_hand_p=x0;
    def_hand_o=o0;
    
    cout << "Default hand position set: " << def_hand_p.toString().c_str() << endl;
    cout << "Default hand orientation set: " << def_hand_o.toString().c_str() << endl;
    
}

void HandTactileControlThread::setArm(Vector armConf)
{
    ctrlMode->getControlModes(controlModes);
    for(int i=0; i<nAxes; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION)
	{
	    ctrlMode->setControlMode(i,VOCAB_CM_POSITION);
	}
    }
    
    for (int i= 0; i < nAxes; i++)
    {
	armFullConf[i] = armConf[i];
    }
	    
    pos->positionMove(armFullConf);  // position move command using bell-shaped velocities, non-blocking
    
    /********** COMMENT THIS TO MAKE IT NON-BLOCKING ***********/
    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();
    while(!done && elapsedTime<5.0)
    {
        pos->checkMotionDone(&done);
	Time::delay(0.04);
	elapsedTime= Time::now()-startTime;
    }
    /***********************************************************/
    
    pos->stop();
    Time::delay(0.5);
    
    Vector x0,o0;
    cartCtrl->getPose(x0,o0);
    
    def_hand_p=x0;
    def_hand_o=o0;
    
    cout << "Default hand position set: " << def_hand_p.toString().c_str() << endl;
    cout << "Default hand orientation set: " << def_hand_o.toString().c_str() << endl;
    
}

void HandTactileControlThread::setDefHandPoseToThis()
{   
    Vector x0,o0;
    cartCtrl->getPose(x0,o0);
    
    def_hand_p=x0;
    def_hand_o=o0;
    
    cout << "Default hand position set: " << def_hand_p.toString().c_str() << endl;
    cout << "Default hand orientation set: " << def_hand_o.toString().c_str() << endl;
    
}

void HandTactileControlThread::handMoveToPose(Vector xd, Vector od)
{
    Vector x0,o0;
    cartCtrl->getPose(x0,o0); 
    
    cout << "Old hand position: " << x0.toString().c_str() << endl;
    cout << "Old hand orientation: " << o0.toString().c_str() << endl;
    
    cout << "Next hand position: " << xd.toString().c_str() << endl;
    cout << "Next hand orientation: " << od.toString().c_str() << endl;
    
    cout << "Executing movement..." << endl;
    
    cartCtrl->goToPoseSync(xd,od);   // send request and wait for reply
    cartCtrl->waitMotionDone(0.04);  // wait until the motion is done and ping at each 0.04 seconds
    
    cartCtrl->getPose(x0,o0); 
    
    cout << "Current hand position: " << x0.toString().c_str() << endl;
    cout << "Current hand orientation: " << o0.toString().c_str() << endl;
  
}



bool HandTactileControlThread::readFingerSkinCompData(bool block)  
{

	Vector *iCubSkinData = portSkinCompIn->read(block);  // block is typically false...
	//fprintf(stderr,"\n\n\n\n\n\n\n\n\nReceived a tactile vector of size = %i\n\n\n\n\n\n\n\n", (int)(iCubSkinData->size()));
	
    
	//TODO generalize fingers number
        if (iCubSkinData) 
	{
		//fprintf(stderr,"\n");
		//fprintf(stderr,"\n");
		//fprintf(stderr,"FINGERS TAXELS:");
		//fprintf(stderr,"\n");
		
	        for(int i = 0; i < N_FINGERS; i++)
		{
			for (int j = 0; j < N_TAXELS; j++)
			{
			        fingerTaxelsDataBinary(i,j)=0.0;
				fingerTaxelsData(i,j) = fingersSensitivityScale(i,0) * (*iCubSkinData)[12*i + j];
				
				if (fingerTaxelsData(i,j) > TOUCH_THR)
				{
				    fingerTaxelsDataBinary(i,j)=1.0;
				}
				//fprintf(stderr,"%.1lf  ", (*iCubSkinData)[12*i + j]);
				//fprintf(stderr,"%.1lf  ", fingerTaxelsDataBinary(i,j));
			}
			//fprintf(stderr,"\n");
		}
		
		//fprintf(stderr,"\n");
		//fprintf(stderr,"\n");
	}

	return true;
}


void HandTactileControlThread::updateFingertipForces()
{

	std::vector<double> unitVector(3);
	std::vector<double> tmpUV(3);
	std::vector<double> newUV(3);
	std::vector<double> prevUV(3);
	
	//fprintf(stderr,"\n1\n");
	
	for (int i = 0; i < N_FINGERS; i++)   // compute unitary forces applied on the fingertips, in the fingertip reference frame
	{
	    
	    //fprintf(stderr,"\n2_%i\n", i);
	    for (int k=0; k<3; k++)
	    {
	        newUV[k] = 0.0;
	        prevUV[k] = 0.0;
	    }
	
	    for(int j = 0; j < N_TAXELS; j++)
	    {
		    
		    //fprintf(stderr,"\n3_%i\n", j);
		    
	            getUnitVector(j,unitVector);

		    tmpUV[0] = fingerTaxelsDataBinary(i,j)*unitVector[0];
		    tmpUV[1] = fingerTaxelsDataBinary(i,j)*unitVector[1];
		    tmpUV[2] = fingerTaxelsDataBinary(i,j)*unitVector[2];
		    
		    sumUnitVectors(prevUV, tmpUV, newUV);
		    
		    for (int k=0; k<3; k++)
		    {
			prevUV[k] = newUV[k];
		    }    		    
		    
	    }
	    
	    for (int k=0; k<3; k++)
	    {
		fingertipForcesLocal(i,k) = newUV[k];
	    }    
	    
	}
	
	// transform the fingertips forces to the hand palm reference frame
	
	Vector fingAngs;
	Vector fingPos;
	Matrix tipFrame(4,4);
	
	//fprintf(stderr,"\n4\n");
	
	for (int i = 0; i < N_FINGERS; i++)   // compute unitary forces applied on the fingertips, in the fingertip reference frame
	{
	    //fprintf(stderr,"\n%s\n",fingers[i].getType().c_str()); 
	    //tipFrame.zero();
	    fingAngs.clear();
	    encs->getEncoders(encoders.data());
	    fingers[i].getChainJoints(encoders,fingAngs);                // wrt the end-effector frame
	    tipFrame=fingers[i].getH((M_PI/180.0)*fingAngs);
	    
	    //fprintf(stderr,"\n5_%i\n",i);
	       
	    for (int k=0; k<3; k++)
	    {
	        //fprintf(stderr,"\n6_%i\n",k);
	        fingertipForcesGlobal(i,k) = tipFrame(k,0)*fingertipForcesLocal(i,0) + tipFrame(k,1)*fingertipForcesLocal(i,1) + tipFrame(k,2)*fingertipForcesLocal(i,2);
	    }
	    
	}
	    
	//fprintf(stderr,"\n6\n");
	
	//Vector tip_x=tipFrame.getCol(3);
	//Vector tip_o=yarp::math::dcm2axis(tipFrame);
	//cartCtrl->attachTipFrame(tip_x,tip_o);                // establish the new controlled frame
	//cartCtrl->getPose(fingPos,o);                             // so as the target will be attained with the finger tip
	//cartCtrl->removeTipFrame();   
	    
}

void HandTactileControlThread::sumUnitVectors(std::vector<double>& A, std::vector<double>& B, std::vector<double>& res)
{
    
    for (int i=0; i<3; i++)
    {
        res[i] = A[i] + B[i];
    }
    
    double norm = sqrt(res[0]*res[0] + res[1]*res[1] + res[2]*res[2]);
    
    if (norm==0)
    {
        norm=0.0000000001;
    }
    
    for (int i=0; i<3; i++)
    {
        res[i] = res[i] / norm;
    }
    
    
}
    

void HandTactileControlThread::getUnitVector(int index,std::vector<double>& unitVector)
{

	switch(index)
	{

	case 0:
		unitVector[0] = -1.0;
		unitVector[1] = 0.0;
		unitVector[2] = 0.0;
		break;

	case 1:
		unitVector[0] = -0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;

	case 2:
		unitVector[0] = -0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;

	case 3:
		unitVector[0] = -1.0;
		unitVector[1] = 0.0;
		unitVector[2] = 0.0;
		break;

	case 4:
		unitVector[0] = -0.78673;
		unitVector[1] = 0.60316;
		unitVector[2] = 0.13140;
		break;

	case 5:
		unitVector[0] = -0.30907;
		unitVector[1] = 0.47765;
		unitVector[2] = 0.82239;
		break;

	case 6:
		unitVector[0] = 0.0;
		unitVector[1] = 1.0;
		unitVector[2] = 0.0;
		break;

	case 7:
		unitVector[0] = 0.30907;
		unitVector[1] = 0.47765;
		unitVector[2] = 0.82239;
		break;

	case 8:
		unitVector[0] = 0.78673;
		unitVector[1] = 0.60316;
		unitVector[2] = 0.13140;
		break;

	case 9:
		unitVector[0] = 1.0;
		unitVector[1] = 0.0;
		unitVector[2] = 0.0;
		break;

	case 10:
		unitVector[0] = 0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;

	case 11:
		unitVector[0] = 0.39956;
		unitVector[1] = 0.0;
		unitVector[2] = 0.91671;
		break;
	}

}

void HandTactileControlThread::updateRef()
{
  
    fprintf(stderr,"\nUpdating target reference...\n");
	
    for (int i=0; i<nParameters; i++)  //Cartesian Y and Z position
    {
	target_hand_p[i+1] = def_hand_p[i+1] + ctrl_param[i]; 
    }
    
    if (nParameters<3)    //Cartesian X position
    {
	target_hand_p[0] = def_hand_p[0];
    }
    
    for (int i=0; i<4; i++)
    {
	target_hand_o[i] = def_hand_o[i]; 
    }
    
    cout << "Target hand position set: " << target_hand_p.toString().c_str() << endl;
    cout << "Target hand orientation set: " << target_hand_o.toString().c_str() << endl;
    
    fprintf(stderr,"...done.\n");
    
}

void HandTactileControlThread::openHand()   
{
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION);
	}
    }
    
    for(int i=0; i<ctrlJoints; i++)
    {
        //handPosRef[i]=HAND_DEF_HOME[i];
	handPosRef[i] = (*armRestPos)[i+7];
    }
	    
    pos->positionMove(ctrlJoints, controlledJoints, handPosRef);  // position move command using bell-shaped velocities, non-blocking
    
}

void HandTactileControlThread::stopFinger(int index)
{
    int cMode3[3];
    int js3[3];
    int cMode2[2];
    int js2[2];
    int cMode1[1];
    int js1[1];
    
    switch(index)
	{

	case 4: //thumb
	        
		js3[0]=8;
		js3[1]=9;
		js3[2]=10;
		
	        ctrlMode->getControlModes(3, js3, cMode3);
                for (int i=0; i<3; i++)		
		{
		    if (cMode3[i]!=VOCAB_CM_POSITION)
		    {
		        ctrlMode->setControlMode(js3[i],VOCAB_CM_POSITION);
		    }
		}
		
		pos->stop(8);
		pos->stop(9);
		pos->stop(10);
		
		break;

	case 0: //index
	  
                js2[0]=11;
		js2[1]=12;
		
	        ctrlMode->getControlModes(2, js2, cMode2);
                for (int i=0; i<2; i++)		
		{
		    if (cMode2[i]!=VOCAB_CM_POSITION)
		    {
		        ctrlMode->setControlMode(js2[i],VOCAB_CM_POSITION);
		    }
		}	  
	  
		pos->stop(11);
		pos->stop(12);
		
		break;

	case 1: //middle
	  
	        js2[0]=13;
		js2[1]=14;
		
	        ctrlMode->getControlModes(2, js2, cMode2);
                for (int i=0; i<2; i++)		
		{
		    if (cMode2[i]!=VOCAB_CM_POSITION)
		    {
		        ctrlMode->setControlMode(js2[i],VOCAB_CM_POSITION);
		    }
		}
	  
		pos->stop(13);
		pos->stop(14);
		break;

	case 2: //ring
	  
	        js1[0]=15;
		
	        ctrlMode->getControlModes(1, js1, cMode1);
                for (int i=0; i<1; i++)		
		{
		    if (cMode1[i]!=VOCAB_CM_POSITION)
		    {
		        ctrlMode->setControlMode(js1[i],VOCAB_CM_POSITION);
		    }
		}
	  
		pos->stop(15);
		break;

	case 3: //little
	  
	        js1[0]=15;
		
	        ctrlMode->getControlModes(1, js1, cMode1);
                for (int i=0; i<1; i++)		
		{
		    if (cMode1[i]!=VOCAB_CM_POSITION)
		    {
		        ctrlMode->setControlMode(js1[i],VOCAB_CM_POSITION);
		    }
		}  
	
		pos->stop(15);
		break;

	}
      
}

void HandTactileControlThread::checkFingersContacts(bool *contactsList)
{
    double avgPressure;
    for (int i=0;i<N_FINGERS;i++)
    {  
        avgPressure = 0.0;
        for (int j=0;j<N_TAXELS;j++)
	{
	    avgPressure += fingerTaxelsDataBinary(i,j);
	}
	if (avgPressure > 0.0)
	{
	    contactsList[i] = true;
	    fprintf(stderr,"\n\n\nDetected contact on finger %i. Average taxles pressure is = %.2lf.\n\n\n", i, avgPressure);
	}
    }
}

void HandTactileControlThread::squeezingStep()   
{
     /********* POSTION DIRECT  *********
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION_DIRECT)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION_DIRECT);
	}
    }
    
    encs->getEncoders(encoders.data());
    
    fprintf(stderr,"\nClose the fingers a bit more...\n");
    
    for (int i=8; i<16; i++)
    {  
        posDir->setPosition(i, encoders[i] + HAND_SQUEEZE_STEP[i-7]);  // position move command, non-blocking
    }
    
    /*******************************/
    
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION);
	}
    }
    
    encs->getEncoders(encoders.data());
    
    fprintf(stderr,"\nClose the fingers a bit more...\n");
    
    for (int i=8; i<16; i++)
    {  
        pos->positionMove(i, encoders[i] + HAND_SQUEEZE_STEP[i-7]);  // position move command, non-blocking
    }
    
    Time::delay(1.0);
    fprintf(stderr,"\n...done.\n");
}

void HandTactileControlThread::closeHandToContact()   
{
    /********* POSTION DIRECT  *********
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION_DIRECT)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION_DIRECT);
	}
    }
    
    for(int i=0; i<ctrlJoints; i++)
    {
        handPosRef[i]=HAND_DEF_TARGET[i];
    }
    
    posDir->setPositions(ctrlJoints, controlledJoints, handPosRef);  // position move command, non-blocking
    
    /**********************************/
    
    ctrlMode->getControlModes(ctrlJoints, controlledJoints, controlModes);
    for(int i=0; i<ctrlJoints; i++)
    {
        if (controlModes[i]!=VOCAB_CM_POSITION)
	{
	    ctrlMode->setControlMode(controlledJoints[i],VOCAB_CM_POSITION);
	}
    }
    
    for(int i=0; i<ctrlJoints; i++)
    {
        handPosRef[i]=HAND_DEF_TARGET[i];
    }
    
    pos->positionMove(ctrlJoints, controlledJoints, handPosRef);  // position move command, non-blocking
    
    bool contacts[5]; //contacts on each fingertip
    for(int i=0; i<5; i++)
    {
        contacts[i]=false;
    }
    bool allContacts=false;
    bool posMoveDone=false;
    double startTime = Time::now();
    
    readEncs();
    
    fprintf(stderr,"\nCurrent fingers configuration is: \n");
    for(int i=0; i<ctrlJoints; i++)  
    {
        fprintf(stderr,"%.2lf  ", encoders[i+7]);
    }
    fprintf(stderr,"\nTarget fingers configuration is: \n");
    for(int i=0; i<ctrlJoints; i++)  
    {
        fprintf(stderr,"%.2lf  ", handPosRef[i]);
    } 
    
    fprintf(stderr,"\nStart to close the fingers...\n");
    
    while (!posMoveDone && !allContacts && ((Time::now() - startTime) < MAX_GRASP_DURATION) )
    {
        readEncs();
        readFingerSkinCompData();
	checkFingersContacts(contacts);
	allContacts=true;                      //optimistic robot!
	for (int i=0;i<N_FINGERS;i++)          //then checking the above...
	{
	    if (contacts[i])
	    {
	        stopFinger(i);
		fprintf(stderr,"\nStop finger %i because of contact detected.\n", i);
	    }
	    allContacts = allContacts && contacts[i];
	}
	checkMotionDone_posDir_handGrasp(posMoveDone);  //VERY SPECIFIC... MIGHT NOT WORK IN GENERAL SITUATIONS...
	if (posMoveDone)
	{
	    fprintf(stderr,"\n\n--TARGET FINGERS CONFIGURATION REACHED--\n\n");
	}
	if (((Time::now() - startTime) >= (MAX_GRASP_DURATION - 0.05)))
	{
	    fprintf(stderr,"\n\n--MOVEMENT TIME DEADLINE EXPIRING--\n\n");
	}
	 
	Time::delay(0.01);
    }
    
    fprintf(stderr,"\n...done.\n");
    
    readEncs();
    
    fprintf(stderr,"\nCurrent fingers configuration is: \n");
    for(int i=0; i<ctrlJoints; i++)  
    {
        fprintf(stderr,"%.2lf  ", encoders[i+7]);
    }
    fprintf(stderr,"\nTarget fingers configuration was: \n");
    for(int i=0; i<ctrlJoints; i++)  
    {
        fprintf(stderr,"%.2lf  ", handPosRef[i]);
    } 
    
}

void HandTactileControlThread::checkMotionDone_posDir_handGrasp(bool &done)
{
    done=true; //optimistic robot!
    readEncs();
    
    for(int i=0; i<ctrlJoints; i++)  //now checking the above...
    {
        if ( fabs(encoders[i+7] - handPosRef[i]) > GRASP_POS_THR)
	{
	    done=false;
	}
    } 
     
}

void HandTactileControlThread::computeGraspMetric()
{
    fprintf(stderr,"Read tactile data...\n");
    readFingerSkinCompData(false); //blocking (on receiving tactile data from port)
    fprintf(stderr,"...done.\n");
    fprintf(stderr,"Compute grasp metric...\n");
    updateFingertipForces();      //computes the unit vector forces applied to the fingertip, in both local and global coordinates
    fprintf(stderr,"...done.\n");
    
    fprintf(stderr,"\n");
    fprintf(stderr,"Local unit vector forces on fingertips\n");
    fprintf(stderr,"\n");
    for (int i = 0; i < N_FINGERS; i++)
    {
        for (int k = 0; k < 3; k++)
	{
	    fprintf(stderr,"%.1lf  ", fingertipForcesLocal(i,k));
	}
	fprintf(stderr,"\n");
    }
    fprintf(stderr,"\n");
    
    fprintf(stderr,"\n");
    fprintf(stderr,"Global unit vector forces on fingertips\n");
    fprintf(stderr,"\n");
    for (int i = 0; i < N_FINGERS; i++)
    {
        for (int k = 0; k < 3; k++)
	{
	    fprintf(stderr,"%.1lf  ", fingertipForcesGlobal(i,k));
	}
	fprintf(stderr,"\n");
    }
    fprintf(stderr,"\n");
    
    /********* ACTUAL GRASP METRIC IS CURRENTLY COMPUTED OUTSIDE THIS MODULE (based on the global unit vector forces) ***********/
  
  
}

void HandTactileControlThread::readData() //non-blocking
{
    readEncs(); //non-blocking
    readFingerSkinCompData(); //non-blocking
}
    

void HandTactileControlThread::readEncs(bool v)
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

bool HandTactileControlThread::readInputPort()
{
    
    if(Bottle *bot=inputPort->read(false))  // when control parameters (e.g. target hand pose) are received
    {   
        if (bot->size() != nParameters)
	{
            cout << "\n[WARNING] - Problem in size of packet received from optimization engine...\n";
	    cout << bot->size() << " values received, " << nParameters << " expected.\n";
        }
    
        for (int i=0; i<nParameters; i++)
	{
	    ctrl_param[i]=bot->get(i).asDouble(); 
	}
	
	fprintf(stderr,"\nNew set of control parameters received\n");
	fprintf(stderr,"%i parameters were received\n", bot->size());
	
	return true;
    }

    return false;
  
}


void HandTactileControlThread::dumpData()
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

void HandTactileControlThread::sendData()
{
    Bottle &outBottle = outputPort->prepare();
    outBottle.clear();

    for(int i=0; i<N_FINGERS; i++)
    {
        for(int k=0; k<3; k++)
	{
	    outBottle.addDouble(fingertipForcesGlobal(i,k));
	    //outBottle.addDouble(fingertipForcesLocal(i,k));
	    //outBottle.addDouble(fingerTaxelsData(i,k));
	   
	}
    }

    outputPort->write(); //sends the unit vector forces applied on the fingertips ()
  
}


void HandTactileControlThread::run()
{
    ctrlStep++;  
  
    if (ctrlStep%DISPLAY_RATE==0)
    {  
        fprintf(stderr,"\n\nCycle time (real control loop time): \n");
	fprintf(stderr,"%.4lf\n\n",Time::now()-cycleTime);
    }
    
    cycleTime=Time::now();
    
    readData();
    if (readInputPort())
    {
        updateRef();
	controlMode = 2;
    }
   
    
    switch (controlMode)
    {
      
    case 0:
       //pos->stop();
       fprintf(stderr, "\n\n-- no hand control (idle)--\n\n");
       Time::delay(0.5);
       break;
       
    case 1:
       //pos->stop();
       fprintf(stderr, "\n\n-- waiting for a new target hand pose --\n\n");
       Time::delay(0.1);
       break;
	
    case 2:
       openHand(); // Non blocking.
       handMoveToPose(target_hand_p, target_hand_o); // Blocking.
       closeHandToContact(); // Checking tactile sensors inside here. Stop each finger after contact. Blocking. 
       //squeezingStep(); // Close the fingers a bit more to improve grasp robustness.
       computeGraspMetric(); // Based on final touch configuration from previous step.
       sendData(); // Send grasp metric to optimization engine.
       dumpData(); // Dump relevant data.
       openHand(); // Non blocking.
       controlMode=1;
       break;
       
    case 3:
       //pos->stop();
       computeGraspMetric(); // Based on fingertip contacts.
       sendData(); // Send grasp metric to optimization engine.
       Time::delay(0.1);
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
        /*********
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
        /******************/
    }
    
}


void HandTactileControlThread::threadRelease() 
{    
    delete[] controlledJoints;
    delete[] controlModes;
    delete[] handPosRef;
    delete[] handVelRef;
    delete[] armFullConf;
}


