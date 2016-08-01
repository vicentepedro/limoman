// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright: (C) 2015 Vis Lab - Instituto Superior Tecnico
// Authors: Lorenzo Jamone (lorejam)
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "gloveToYarp.h"


const int JOINT_POS_SENSORS_N = 14;

const int DEFAULT_CONTROL_PERIOD = 20;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */
bool GloveToYarpModule::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */
    cout << "Processing program parameters\n";
    

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                           Value("gloveToYarp"), 
                           "module name (string)").asString();

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());
    
    moduleLocalPortName           = "/" ;
    moduleLocalPortName           += getName( rf.check("localPort", 
                               Value("/local"),
                               "Output joint values port (string)").asString()
                               );                       
    outPortName          = "/" ;
    outPortName          += getName( rf.check("outPort", 
                               Value("/jointsPos:o"),
                               "Output port to send hand joints values").asString()
                               );

    setPortName          = "/" ;
    setPortName          += getName( rf.check("setPort", 
                               Value("/set:rpc"),
                               "Rpc port to set parameters").asString()
                               );

    gloveLocalPortName   = rf.check("gloveLocalPortName", 
                           Value("/dev/rfcomm0"), 
                           "Local port to which the glove is connected (string)").asString();

    gloveCalibFileName   = rf.check("gloveCalibFileName", 
                           Value("glove.calib"), 
                           "Glove calibration file").asString();
        
    if( rf.check("defaultValues", "Rest position of the head") )
        defValsBottle = rf.findGroup("defaultValues").tail();
                              
    period                  = rf.check("period", 
                              Value(DEFAULT_CONTROL_PERIOD), 
                              "Thread period").asInt();

    /* do all initialization here */
    
    /* open ports  */ 
    
    if (!outPort.open(outPortName.c_str())) 
    {
        cout << getName() << ": unable to open port " << outPortName.c_str() << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!setPort.open(setPortName.c_str())) 
    {
        cout << getName() << ": unable to open port " << setPortName.c_str() << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    nAxes = JOINT_POS_SENSORS_N;
    defVals = new double[nAxes];

    for (int i= 0; i < nAxes; i++)
    {
        defVals[i]=defValsBottle.get(i).asDouble();
    }

    /* create glove object  */

    glove = new LocalCyberGlove(gloveLocalPortName.c_str());

    /* connect to the glove */

    if (glove->connect()) 
    {

        cout << glove->info() << endl;
        pair<int, int> version = glove->version();
        cout << "Version: " << version.first << "." << version.second << endl << endl;
        glove->setCalibration(new CyberGloveLinearCalibration(gloveCalibFileName.c_str()));
    }
    else
    {
        cout << getName() << ": unable to connect to local device [CyberGlove(R)] " << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
        
    /* create the thread and pass pointers to the module parameters */

    gloveToYarpThread = new GloveToYarpThread(glove, &outPort, period, defVals);

    /* now start the thread to do the work */

    gloveToYarpThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    
    cmdT = new UserCmdThread(gloveToYarpThread,&setPort);
    cmdT->start();

    return true ;      // let the RFModule know everything went well
                       // so that it will then run the module
}

bool GloveToYarpModule::interruptModule()
{
    outPort.interrupt();
    return true;
}


bool GloveToYarpModule::close()
{
    outPort.close();
    setPort.close();

    /* stop the thread */
    gloveToYarpThread->stop();
    cmdT->stop();

    glove->disconnect();

    delete gloveToYarpThread;
    delete cmdT;
    delete glove;

    return true;
}

/* Called periodically every getPeriod() seconds */ 

bool GloveToYarpModule::updateModule()
{
   return true;
} 

double GloveToYarpModule::getPeriod()
{
   /* module periodicity (seconds), called implicitly by myModule */
   return 0.5;
}

//**********************************
  
UserCmdThread::UserCmdThread(GloveToYarpThread *g, RpcServer *s)
{
    g2yThread=g;
    setPort=s; 
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
		fprintf(stderr,"Desired module mode is: %d\n", tmpI);
	        fprintf(stderr,"Current module mode is: %d\n", g2yThread->mode);
	        g2yThread->mode = tmpI;
	        fprintf(stderr,"New control mode is: %d\n", g2yThread->mode);
		rep.addString("done");
		rep.addInt(g2yThread->mode);
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
		rep.addInt(g2yThread->mode);
	        setPort->reply(rep);
		fprintf(stderr,"Control mode sent is: %d\n", g2yThread->mode);
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


GloveToYarpThread::GloveToYarpThread(LocalCyberGlove *g, BufferedPort< Bottle > *p, int per, double *df) : RateThread(per)
{
    glove  = g;
    outPort   = p;    
    periodInMsec = per;
    periodInSec = (double)per/1000.0;
    iteration = 0;
    displayRate = 500;
    sendBufferIndex = 0;
    gloveBufferId = 0;

    mode=1;   // 0 - idle ; 1 - send ; 2 - dump ; 3 - send & dump
    
    read=false;

    nAxes = JOINT_POS_SENSORS_N;
    jointsPos = new double[nAxes];
    tmpJointVals = new double[nAxes];

    defJointVals = df;
    
} 

bool GloveToYarpThread::threadInit() 
{
    cout << endl << "Verifying glove status..." <<endl;
    cout << glove->info() << endl;
    pair<int, int> version = glove->version();
    cout << "Version: " << version.first << "." << version.second << endl << endl;
    cout << endl << "...ok!" << endl;

    
    for(int i = 0; i < nAxes; i++) 
    {
	    jointsPos[i] = defJointVals[i];
    }
    
    Time::delay(0.1);

    fprintf(stderr,"\n\nGloveToYarpThread -- Thread started!\n\n");

    //cout << "Start moving!\n" << endl;
    // Start moving
    //moveRandomPos();
    
    startTime=Time::now();
    cycleTime=0.0;

    return true;
}

void GloveToYarpThread::readDataFast()
{
    try
    {
        sensValues = glove->getAngles(true);
    }
    catch (invalid_argument& e) 
    {
        cout << endl;
        cerr << e.what() << endl;
    }

    sendBufferIndex = 0;

    for (int i=0; i<20; i++)
    {
        if (i!=0 && i!=6 && i!=7 && i!=10 && i!=14 && i!=18)
        {
            jointsPos[sendBufferIndex]=sensValues[i];
            sendBufferIndex++;
        }
    }

}


void GloveToYarpThread::readData()
{  
    sendBufferIndex = 0;

    for (int f = CyberGlove::THUMB; f <= CyberGlove::PINKY; f++) 
    {
        for (int j = CyberGlove::METACARPAL; j <= CyberGlove::ABDUCTION; j++) 
        {
            
            try
            {
                CyberGlove::Finger finger = (CyberGlove::Finger) f;
                CyberGlove::Joint joint = (CyberGlove::Joint) j;

                gloveBufferId = glove->getSensorId(finger, joint);
            
                if (gloveBufferId!=0 && gloveBufferId!=6 && gloveBufferId!=7 && gloveBufferId!=10 && gloveBufferId!=14 && gloveBufferId!=18)
                {
                    jointsPos[sendBufferIndex] =  glove->getAngle(finger, joint, true);
                    sendBufferIndex++;
                }
            }
            catch (invalid_argument& e) 
            {
                //cout << endl;
                //cerr << e.what() << endl;
            }
        }
    }    
            
}

void GloveToYarpThread::remapData()
{

    for(int i = 0; i < nAxes; i++)
    {
        tmpJointVals[i]=jointsPos[i];
    }

    jointsPos[0] = tmpJointVals[2];  // thumb abduction/opposition
    jointsPos[1] = tmpJointVals[0];  // thumb proximal
    jointsPos[2] = tmpJointVals[1];  // thumb distal
    jointsPos[3] = tmpJointVals[3];  // index proximal
    jointsPos[4] = tmpJointVals[4];  // index distal
    jointsPos[5] = tmpJointVals[5];  // middle proximal
    jointsPos[6] = tmpJointVals[6];  // middle distal
    jointsPos[7] = tmpJointVals[8];  // ring proximal
    jointsPos[8] = tmpJointVals[9];  // ring distal
    jointsPos[9] = tmpJointVals[11];  // pinky proximal
    jointsPos[10] = tmpJointVals[12];  // pinky distal
    jointsPos[11] = tmpJointVals[7];  // index-middle abduction
    jointsPos[12] = tmpJointVals[10];  // middle-ring abduction
    jointsPos[13] = tmpJointVals[13];  // ring-pinky abduction
            
}

void GloveToYarpThread::sendData()
{

    Bottle &outBottle = outPort->prepare();
    outBottle.clear();

    for(int i = 0; i < nAxes; i++)
    {
        outBottle.addDouble(jointsPos[i]);
    }

    outPort->write();
            
}

void GloveToYarpThread::dumpData()
{
   
            
}


void GloveToYarpThread::initData()
{
    
  
}


void GloveToYarpThread::run()
{
    iteration++;

    if (iteration%displayRate==0)
    {  
        fprintf(stderr,"\n\nCycle time (real RateThread loop time): \n");
	    fprintf(stderr,"%.4lf\n\n",Time::now()-cycleTime);
    }
    
    cycleTime=Time::now();
    
    switch (mode)
    {
      
    case 0:
       fprintf(stderr, "\n\n-- IDLE --\n\n");
       Time::delay(0.1);
       break;
	
    case 1:
       //readData();
       readDataFast();
       remapData();
       sendData();
       break;
	
    case 2: 
       //readData();
       readDataFast();
       remapData();
       dumpData();
	   break;
		
	case 3:
	   //readData();
       readDataFast();
       remapData();
       sendData();
       dumpData();
	   break;
		
    default:
	   fprintf(stderr, "\n\n-- IDLE (check module mode...) --\n\n");
	   Time::delay(1.0);
	   break;
		
	}
       
    if (iteration%displayRate==0)
    {  
        fprintf(stderr,"\n\nCycle computation time: \n");
	    fprintf(stderr,"%.4lf\n\n",Time::now()-cycleTime);
    }
    
    
}


void GloveToYarpThread::threadRelease() 
{
    delete jointsPos;
}


