#include "gta_robot.h"
#include "htmlControl.h"

extern HTML510Server htmlServer;

GTARobot::GTARobot()
    : m_robo_state(MANUAL)
{}

GTARobot::~GTARobot()
{}

// This function will perform all the data processing neccessary for each tick
void GTARobot::processTick()
{
    // html running
    // static uint32_t lastWebCheck = millis();
    // uint32_t time_now = millis();

    // if (time_now - lastWebCheck >= 10){ 
    //     //htmlServer.serve(server,body);  // WiFiServer object in the brackets
    //     htmlServer.serve();
    //     lastWebCheck = now;
    // }

    htmlServer.serve();

    handleCanMsg();
    handleRobotMsg();

    switch(m_robo_state)
    {
        case MANUAL:
            moveRobot();
            break;
        case AUTONOMOUS:
            moveToGivenPos();
            break;
        case WALL_FOLLOWING:
            wallFollowing();
            break;
        case BEACON_SENSING:
            beaconSensing();
            break;
        default:
            break;
    }
}

void GTARobot::moveRobot()
{
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

    rl.calculate_wheel_vel();
    rl.updateDirection();
    rl.updatePWM();
}

void  GTARobot::wallFollowing(){
    static uint64_t previous_time;

    // readings from three ulreasonic sensors in [cm]
    int sensing_dist[ULTRASOIC_SENSOR_NUM];
    if (millis() - previous_time >= 60){
        for(int i = 0; i < ULTRASOIC_SENSOR_NUM; i++){
        sensing_dist[i] = hcsr04.dist(i);
        }
        previous_time = millis();
    }
    // reasoning behavior

    // the robot is far from the wall
    if(sensing_dist[0] >= 50 && sensing_dist[2] >= 50){

        if(sensing_dist[1] > FRONT_DIST){
        // front is far from the wall, turn slowly
        rl.turnRight();
        }

        else if(sensing_dist[1] <= FRONT_DIST){
        // front is close the wall, turn quickly
        rl.turnRight(0.7);
        }
    }

    // counter-clockwise
    else if(sensing_dist[0] >= 50 && sensing_dist[2] <= 50){

        if(sensing_dist[1] > FRONT_DIST){
        // the robot following a wall, turn slowly
        if(sensing_dist[2] < LATERAL_DIST - 1.f){
            rl.turnLeft();
        }
        else if(abs(LATERAL_DIST - sensing_dist[2]) <= 1.f){
            rl.goStraight(0.7);
        }
        else if(sensing_dist[2] > LATERAL_DIST + 1.f){
            rl.turnRight();
        } 
        }

        else if(sensing_dist[1] <= FRONT_DIST){
        // the robot in the corner, turn quickly
            rl.turnLeft(0.7);
        }
    }
    
    // clockwise
    else if(sensing_dist[0] <= 50 && sensing_dist[2] >= 50){

        if(sensing_dist[1] > FRONT_DIST){
        // the robot following a wall, turn slowly
        if(sensing_dist[0] < LATERAL_DIST - 1.f){
            rl.turnRight();
        }
        else if(abs(LATERAL_DIST - sensing_dist[0]) <= 1.f){
            rl.goStraight(0.7);
        }
        else if(sensing_dist[0] > LATERAL_DIST+ 1.f){
            rl.turnLeft();
        } 
        }

        else if(sensing_dist[1] <= FRONT_DIST){
        // the robot in the corner, turn quickly
            rl.turnRight(0.7);
        }
    }
}

void GTARobot::beaconSensing(){

}

void GTARobot::setRoboID(){
    roboID = (digitalRead(DIP_SWITCH_PIN1) << 1) + digitalRead(DIP_SWITCH_PIN2) + 1;
}

void GTARobot::setState(robot_states state)
{
    m_robo_state = state;
}

void GTARobot::viveUDPSetup(){
    Serial.println("Vive trackers started!");

    Serial.print("Connecting to "); Serial.print(ssid);
    WiFi.config(ipLocal1, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
        
    WiFi.begin(ssid, password);

    robotUDPServer.begin(robotUDPPort); // robot UDP port 2510
    canUDPServer.begin(canUDPPort); // can UDP port 1510

    // pullup two dip switch pin
    pinMode(DIP_SWITCH_PIN1, INPUT_PULLUP);
    pinMode(DIP_SWITCH_PIN2, INPUT_PULLUP);

    vive1.begin();
    /* comment it out if you only use one vive circuit */
    vive2.begin();
}

void GTARobot::fncUdpSend(char* datastr, int len){
    robotUDPServer.beginPacket(ipTarget, robotUDPPort);
    robotUDPServer.write((uint8_t*)datastr, len);
    robotUDPServer.endPacket();
}

/* 
Calculate the average positon from the two data points
*/
void GTARobot::calculateAvgPos(int *arr, int x1, int y1, int x2, int y2){
  int avg_x = (x1 + x2)/2;
  int avg_y = (y1 + y2)/2;
  *arr = avg_x;
  *(arr + 1) = avg_y;
}

void GTARobot::posSending(){
    static uint64_t previous_time;
    int x1, y1, x2, y2; // xy position

    setRoboID(); // set robot ID

    if (millis() - previous_time >= 100)
    {
        if(vive1.status()== VIVE_LOCKEDON){
            Serial.printf("X1 %d, Y1 %d\n", vive1.xCoord(), vive1.yCoord());
            Serial.printf("X2 %d, Y2 %d\n", vive2.xCoord(), vive2.yCoord());
        }
        else{
                vive1.sync(15); // try to resync 15 times (nonblocking)
                vive2.sync(15); // try to resync 15 times (nonblocking)
        }
        previous_time = millis();
    }

    if (millis() - previous_time >= 100){
        x1 = vive1.xCoord();
        y1 = vive1.yCoord(); 
        x2 = vive2.xCoord();
        y2 = vive2.yCoord();

        // calculate average position of the 2 data points
        int avgPos[2];
        calculateAvgPos(avgPos, x1, y1, x2, y2);

        // store into a string with format #:####,####, which is robotid, x, y
        //sprintf(s, "%1d:%4d:%4d", 1, x1, y1);
        sprintf(s, "%1d:%4d,%4d", roboID, *avgPos, *(avgPos + 1)); // robot id can be changed
        fncUdpSend(s, 13);
        Serial.printf("sending data acquired by vive: %s\n", s);

        previous_time = millis();
    }
}


void GTARobot::handleCanMsg(){
    const int UDP_PACKET_SIZE = 14;
    uint8_t packetBuffer[UDP_PACKET_SIZE];

    int cb = canUDPServer.parsePacket(); // length of the packet
    if(cb){
        packetBuffer[cb] = 0; // null terminate string

        canUDPServer.read(packetBuffer, UDP_PACKET_SIZE);

        int canID = atoi((char *)packetBuffer); // 1st indexed char
        int x = atoi((char *) packetBuffer+2); // #:####,#### 2nd indexed char
        int y = atoi((char *) packetBuffer+7); // 7th indexed char

        //updateCanData(canID, x, y);
        cans[canID - 1].id = canID;
        cans[canID - 1].x = x;
        cans[canID - 1].y = y;

        Serial.printf("From can %d: %d, %d\n\r", canID, x, y);
    }
}


void GTARobot::handleRobotMsg(){
    const int UDP_PACKET_SIZE = 14;
    uint8_t packetBuffer[UDP_PACKET_SIZE]; 

    int cb = robotUDPServer.parsePacket();  // length of the packet
    if(cb){
        packetBuffer[cb] = 0;  // null terminate string

        robotUDPServer.read(packetBuffer, UDP_PACKET_SIZE);

        int robotID = atoi((char *)packetBuffer); // 1st indexed char
        int x = atoi((char *) packetBuffer+2); // #:####,#### 2nd indexed char
        int y = atoi((char *) packetBuffer+7); // 7th indexed char

        //updateRoboData(robotID, x, y);
        robots[robotID - 1].id = robotID;
        robots[robotID - 1].x = x;   
        robots[robotID - 1].y = y;

        // Serial.printf("From robot %d: %d, %d\n\r", robotID, x, y);
    }
}

void GTARobot::moveToGivenPos(){
    static int canNumToFollow;
    float slope_mypos_to_can;
    float slope_two_vive;
    float slope_multiple;

    int can_x = 0;
    int can_y = 0;

    // assume the line between the middle point of two vive to the can point is ax + by + c = 0, direction from middle point to can
    int a;
    int b;
    int c;
    int d;

    int my_x = robots[roboID - 1].x; // our own robot ID, middle point
    int my_y = robots[roboID - 1].y;

    // determine which can to follow
    for (canNumToFollow = 0; canNumToFollow <= 7; canNumToFollow ++){
        if (cans[canNumToFollow].id != 0){
            can_x = cans[canNumToFollow].x;
            can_y = cans[canNumToFollow].y;
            break;
        }
    }

    slope_mypos_to_can = (float)((my_y - cans[canNumToFollow].y) / (my_x - cans[canNumToFollow].x));
    slope_two_vive = (float)(vive1.yCoord() - vive2.yCoord()) / (vive1.xCoord() - vive2.xCoord());
    slope_multiple = slope_two_vive * slope_mypos_to_can;

    a = can_y - my_y;
    b = my_x - can_x;
    c = can_x * my_y - my_x * can_y;
    d = a * vive1.xCoord() + b * vive1.yCoord() + c;

    // margin remains to tune
    if (abs(slope_multiple - (-1)) <= 0.05 && d < 0)
    {
        rl.goStraight(0.5);
    }
    else {
        rl.turnLeft();
    }
}


