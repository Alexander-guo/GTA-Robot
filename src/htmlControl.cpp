#include "htmlControl.h"
#include "html510.h"
#include "RobotLocomotion.h"

extern GTARobot savage_friday;
const char *body;
HTML510Server htmlServer(80);

float mapFloat(float input, float in_min, float in_max, float out_min, float out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* HTMLControl::HTMLControl(RobotLocomotion rl){
    this->rl = rl;
    //htmlServer.begin();
    HTMLControl::htmlServer = HTML510Server(80);
}
 */

void htmlControlInit(){
    htmlServer.begin(); //Start server

    // writing wrapper func in main.cpp
    htmlServer.attachHandler("/joy?val=",handleJoy);
    htmlServer.attachHandler("/wallFollowingHit",handleWallFollowing);
    htmlServer.attachHandler("/beaconSensingHit",handleBeaconSensing);
    htmlServer.attachHandler("/moveToPosHit", handleMoveToPos);
    htmlServer.attachHandler("/switchmode", handleSwitch);
    body = joybody;     // The initial state of the webpage is displaying the joystick
    
    htmlServer.attachHandler("/favicon.ico",handleFavicon);
    htmlServer.attachHandler("/ ",handleRoot);
}

void handleFavicon(){
    htmlServer.sendplain("");
}
void handleRoot(){
    htmlServer.sendhtml(body);
}

void handleSwitch(){
    // This function is used to switch between manual control and the other funcionalities
    String s = "";
    static int toggle = 0;
    if (toggle)
        body = joybody;
    else
        body = funcbody;
    if (body == joybody)
    {
        savage_friday.setState(MANUAL, IDLE);
    }

    savage_friday.rl.vel.lin_vel = 0;
    savage_friday.rl.vel.anglr_vel = 0;

    toggle = !toggle;
    htmlServer.sendplain(s); //acknowledge
}

void handleJoy(){
    float ang_vel = htmlServer.getVal(); // from -50 to +50
    float lin_vel = htmlServer.getVal();
    
    ang_vel = -mapFloat(ang_vel, -50, 50, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    lin_vel = -mapFloat(lin_vel, -50, 50, -MAX_LINEAR_VEL, MAX_LINEAR_VEL); // defined positive as turn left

    ang_vel *= ANGLR_VEL_SCALING;
    lin_vel *= LIN_VEL_SCALING;

    String s = String(ang_vel) + "rad/s ," + String(lin_vel) + "m/s";
    htmlServer.sendplain(s);

    savage_friday.rl.vel.lin_vel = lin_vel;
    savage_friday.rl.vel.anglr_vel = ang_vel;
}

void handleWallFollowing(){
    // Change the system state to wall following
    savage_friday.setState(WALL_FOLLOWING, IDLE);
}

void handleBeaconSensing(){
    // Change the system state to beacon sensing
    savage_friday.setState(BEACON_SENSING, IDLE);
}

void handleMoveToPos(){
    // Change the system state to autonmous movement to an XY position
    savage_friday.setState(AUTONOMOUS, IDLE);
}
