// Copyright: (C) 2015 Vis Lab - Instituto Superior Tecnico
// Authors: Lorenzo Jamone (lorejam)
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "gloveToYarp.h" 

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
    rf.setDefaultConfigFile("../../../util/gloveToYarp/conf/gloveToYarp.ini"); //overridden by --from parameter
    //rf.setDefaultContext("/home/lorejam/SW/libglove/util/gloveToYarp/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    /* create your module */

    GloveToYarpModule gloveToYarpModule; 

    /* run the module: runModule() calls configure first and, if successful, it then runs */

    gloveToYarpModule.runModule(rf);

    return 0;
}

