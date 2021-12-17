#include "htmlControl.h"
#include "html510.h"

extern GTARobot savage_friday;
const char *body;
HTML510Server htmlServer(80);

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
    savage_friday.setState(MANUAL);
}

void handleWallFollowing(){
    savage_friday.setState(WALL_FOLLOWING);
}

void handleBeaconSensing(){
    savage_friday.setState(BEACON_SENSING);
}

void handleMoveToPos(){
    savage_friday.setState(AUTONOMOUS);
}
