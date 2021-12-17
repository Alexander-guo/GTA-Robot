#include "htmlControl.h"
#include "html510.h"

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
    htmlServer.attachHandler("/beaconSensing",handleBeaconSensing);
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
    toggle = !toggle;
    htmlServer.sendplain(s); //acknowledge
}

void handleJoy(){
    int ang_vel, lin_vel;
    ang_vel = htmlServer.getVal(); // from -50 to +50
    lin_vel = htmlServer.getVal();

    // left = x - y;
    // right = x + y;

    ang_vel = map(ang_vel, -50, 50, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    lin_vel = -map(lin_vel, -50, 50, -MAX_LINEAR_VEL, MAX_LINEAR_VEL); // defined positive as turn left

    String s = String(ang_vel) + "rad/s ," + String(lin_vel) + "m/s";
    htmlServer.sendplain(s);
    //Serial.printf("received X,Y:=%d,%d\n",x,y);

    gtaRobo.rl.calculate_wheel_vel();
    gtaRobo.rl.updateDirection();
    gtaRobo.rl.updatePWM();
}

// TODO(Guo): consider setting flag here to do the corrsponding funtionality, otherwise may not manuveur continuously
void handleWallFollowing(){
    gtaRobo.wallFollowing();
}

void handleBeaconSensing(){
    gtaRobo.beaconSensing();
}

void handleMoveToPos(){
    gtaRobo.movingToPos();
}
