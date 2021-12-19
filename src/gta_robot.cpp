#include "gta_robot.h"
#include "htmlControl.h"
#include <math.h>

#define SERIAL_PLOTTING false
#define SERIAL_LOGGING false
#define USE_MOVING_AVG true

extern HTML510Server htmlServer;

enum robot_movement_states {STOPPED, ADVANCING, TURNING};

GTARobot::GTARobot()
    : m_robo_state(MANUAL)
{
    left_ultrasonic.begin();
    front_ultrasonic.begin();
    right_ultrasonic.begin();
}

GTARobot::~GTARobot()
{}

// This function will perform all the data processing neccessary for each tick
void GTARobot::processTick()
{
    posSending();
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
    rl.calculate_wheel_vel();
    rl.updateDirection();
    rl.updatePWM();
}

void  GTARobot::wallFollowing(){
    // float left_dist = left_ultrasonic.getMedianFilterDistance();    
    float right_dist = right_ultrasonic.getDistance();    
    float front_dist = front_ultrasonic.getDistance();    

    // if (left_dist != HCSR04_OUT_OF_RANGE)
    // {
    //     Serial.printf("Left: %f\n", left_dist);
    // }
    // else
    // {
    //     Serial.println("Left: Out of Range");
    // }

    // if (right_dist != HCSR04_OUT_OF_RANGE)
    // {
    //     Serial.printf("Right: %f\n", right_dist);
    // }
    // else
    // {
    //     Serial.println("Right: Out of Range");
    // }
    // if (front_dist != HCSR04_OUT_OF_RANGE)
    // {
    //     Serial.printf("Front: %f\n", front_dist);
    // }
    // else
    // {
    //     Serial.println("Front: Out of Range");
    // }
    // Serial.printf("left: %d \tfront: %d \tright: %d\n", left_dist, right_dist, front_dist);

    static robot_movement_states robo_movement = STOPPED;
    static uint32_t time_started_turning;

    switch (robo_movement)
    {
    case STOPPED:
        robo_movement = ADVANCING;
        break;
    case ADVANCING:
        if( !(front_dist <= 20 || front_dist == HCSR04_OUT_OF_RANGE))
        {
            if (right_dist < 15 || right_dist == HCSR04_OUT_OF_RANGE)
            {
                rl.turnLeft(0.2, 0.05);   
            }
            else if ( right_dist > 20)
            {
                rl.turnRight(0.2, 0.05);
            }
            else
            {
                rl.goStraight(0.2);
            }
        }
        else
        {
            // Robot has encountered a wall in the front so it stops and turns
            robo_movement = TURNING;
            time_started_turning = millis();
        }
        break;
    case TURNING:
        if ( millis() - time_started_turning <= 350)
        {
            rl.turnLeft(0.0, 0.2);
        }
        else
        {
            robo_movement = ADVANCING;
        }
        break;   
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
    Serial.print("Connecting to ");
    Serial.print(ssid);

    WiFi.mode(WIFI_MODE_STA);
    WiFi.config(ipLocal1, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    WiFi.begin(ssid, password);

    while(WiFi.status() != WL_CONNECTED)
    {
        delay(500); Serial.print(".");
    }

    Serial.println("WIFI Connected");
    Serial.printf("Use this URL: http://%s/\n", WiFi.localIP().toString().c_str());

    robotUDPServer.begin(robotUDPPort); // robot UDP port 2510
    canUDPServer.begin(canUDPPort);     // can UDP port 1510

    // pullup two dip switch pin
    pinMode(SIGNALPIN1, INPUT);
    pinMode(SIGNALPIN2, INPUT);
    pinMode(DIP_SWITCH_PIN1, INPUT_PULLUP);
    pinMode(DIP_SWITCH_PIN2, INPUT_PULLUP);

    vive1.begin();
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
  arr[0] = (x1 + x2)/2;
  arr[1] = (y1 + y2)/2;
}

int GTARobot::calculateMovingAverage(int &last_value, int new_value, int weight)
{
    int update;
    update = (weight * last_value + new_value)/( weight + 1);
    last_value = update;
    return update;
}

void GTARobot::posSending(){
    static uint64_t previous_time;

    setRoboID(); // set robot ID

    if (millis() - previous_time >= 100)
    {
        if(vive1.status() == VIVE_LOCKEDON && vive2.status() == VIVE_LOCKEDON){
            // Serial.printf("X1 %d, Y1 %d\n", vive1.xCoord(), vive1.yCoord());
            // Serial.printf("X2 %d, Y2 %d\n", vive2.xCoord(), vive2.yCoord());

            int x1, y1, x2, y2; // xy position
            x1 = vive1.xCoord();
            y1 = vive1.yCoord(); 
            x2 = vive2.xCoord();
            y2 = vive2.yCoord();

            // calculate average position of the 2 data points
            int avgPos[2];
            calculateAvgPos(avgPos, x1, y1, x2, y2);

            #if USE_MOVING_AVG
            calculateMovingAverage(m_x, avgPos[0], 3);
            calculateMovingAverage(m_y, avgPos[1], 3);
            #else
            m_x = avgPos[0];
            m_y = avgPos[1];
            #endif
            pos_valid = true;
        }
        else{
            m_x = 0;
            m_y = 0;
            pos_valid = false;
        }

        // store into a string with format #:####,####, which is robotid, x, y
        sprintf(s, "%1d:%4d,%4d", roboID, m_x, m_y); // robot id can be changed
        s[11] = 0;  // Null terminate sprintf
        fncUdpSend(s, 13);

        #if SERIAL_PLOTTING
        Serial.print("X:");
        Serial.print(m_x);
        Serial.print("\t");
        Serial.print("Y:");
        Serial.println(m_y);
        #elif SERIAL_LOGGING
        // Serial.printf("sending data acquired by vive: %s\n", s);
        #endif

        if (vive1.status() != VIVE_LOCKEDON)
        {
            vive1.sync(15); // try to resync 15 times (nonblocking)
        }
        if (vive2.status() != VIVE_LOCKEDON)
        {
            vive2.sync(15); // try to resync 15 times (nonblocking)
        }
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

#if 0
    // margin remains to tune
    if (abs(slope_multiple - (-1)) <= 0.2 && d < 0)
    {
        rl.goStraight(0.5);
    }
    else {
        rl.turnLeft(0, 0.2);
    }
#endif

    float angle1 = atan(slope_mypos_to_can);
    float angle2 = atan(slope_two_vive); 
    // margin remains to tune
    if (angle1 >= 0)
    {   
        if (abs((angle1 - PI / 2) - angle2) <= 0.087 && d < 0)
        {
            rl.goStraight(0.5);
        }
        else{
            rl.turnLeft(0, 0.2);
        }
    }
    else if (angle1 < 0){
        if (abs((angle1 + PI / 2) - angle2) <= 0.087 && d < 0){
            rl.goStraight(0.5);
        }
        else{
            rl.turnLeft(0, 0.2);
        }
    }
}


