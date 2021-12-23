#ifndef HTML_CONTROL_H
#define HTML_CONTROL_H

#include "gta_robot.h"
#include "myJoyJS.h"
#include "myWebpage.h"
#include "html510.h"

/*
//Started to implement this as a class, but then realized that it wasn't neccessary
class HTMLControl{

    public:
        static HTML510Server htmlServer;
        static RobotLocomotion rl;

        //const char * ssid = "TP-Link_05AF";
        //const char * password = "47543454";
        
        static const char *body;

        // constructor
        HTMLControl(RobotLocomotion);

        void init();
};
*/

void htmlControlInit(); // initialize html control (after wifi setup)

void handleRoot();
void handleFavicon();
void handleHit();
void handleSwitch(); // Switch between JOYSTICK(manully control) and FUNCTIONALITY mode

/************************/
/* joystick mode  code  */
void handleJoy();

/****************************/
/* funtionality mode  code  */
void handleWallFollowing();
void handleBeaconSensing();
void handleMoveToPos();

#endif