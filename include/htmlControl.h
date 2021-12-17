#ifndef HTML_CONTROL 
#define HTML_CONTROL

#include "gta_robot.h"
#include "myJoyJS.h"
#include "myWebpage.h"
#include "html510.h"

// class HTMLControl{

//     public:
//         static HTML510Server htmlServer;
//         static RobotLocomotion rl;

//        /*  const char * ssid = "TP-Link_05AF";
//         const char * password = "47543454"; */
        
//         static const char *body;

//         // constructor
//         HTMLControl(RobotLocomotion);

//         void init();
// };

const char *body;
HTML510Server htmlServer(80);

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