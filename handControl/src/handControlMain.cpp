#include "../include/handControl.h"

int main(int argc, char * argv[])
{
    /* initialize yarp network */ 

    Network yarp;
    if ( !yarp.checkNetwork() )
    {
        fprintf(stderr,"Connection problem. No yarp server?\n\n");
        return -1;
    }


    /* prepare and configure the resource finder */

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("../conf/handControl.ini"); //overridden by --from parameter
    //rf.setDefaultContext("/home/lorejam/SW/iCub_myApps/imleGazeControl/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    /* create your module */

    HandControlModule handControlModule; 

    /* run the module: runModule() calls configure first and, if successful, it then runs */

    handControlModule.runModule(rf);

    return 0;
}

