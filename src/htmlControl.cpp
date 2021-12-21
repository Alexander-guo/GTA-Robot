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
    /* Serial.begin(115200);
    WiFi.mode(WIFI_MODE_STA); */
    /* WiFi.begin(ssid, password);
    WiFi.config(IPAddress(192, 168, 78, 9), // change the last number to your assigned number
                IPAddress(192, 168, 78, 1),
                IPAddress(255, 255, 255, 0)); */
    /* while(WiFi.status()!= WL_CONNECTED ) { 
        delay(500); Serial.print("."); 
    } */

    htmlServer.begin(); //Start server

    // Serial.println("WiFi connected"); 
    // Serial.printf("Use this URL http://%s/\n",WiFi.localIP().toString().c_str());                

    // writing wrapper func in main.cpp
    htmlServer.attachHandler("/joy?val=",handleJoy);
    htmlServer.attachHandler("/wallFollowingHit",handleWallFollowing);
    htmlServer.attachHandler("/beaconSensingHit",handleBeaconSensing);
    htmlServer.attachHandler("/moveToPosHit", handleMoveToPos);
    htmlServer.attachHandler("/switchmode", handleSwitch);
    //htmlServer.attachHandler("/lever?val=",handleLever);
    body = joybody;
    
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
    String s = "";
    static int toggle = 0;
    if (toggle)
        body = joybody;
    else
        body = funcbody;
    if (body == joybody)
    {
        savage_friday.setState(MANUAL, STOPPED);
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
    savage_friday.setState(WALL_FOLLOWING, STOPPED);
}

void handleBeaconSensing(){
    savage_friday.setState(BEACON_SENSING, STOPPED);
}

void handleMoveToPos(){
    Serial.printf("move to xy!");
    savage_friday.setState(AUTONOMOUS, STOPPED);
}
