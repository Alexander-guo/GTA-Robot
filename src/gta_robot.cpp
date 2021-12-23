#include "gta_robot.h"
#include "htmlControl.h"
#include <math.h>
#include <Arduino.h>

#define SERIAL_PLOTTING false
#define SERIAL_LOGGING false
#define USE_MOVING_AVG true

extern HTML510Server htmlServer;


GTARobot::GTARobot()
    : m_system_state(MANUAL), m_action_state(IDLE)
{
    left_ultrasonic.begin();
    front_ultrasonic.begin();
    right_ultrasonic.begin();
}

GTARobot::~GTARobot()
{}

void GTARobot::viveUDPSetup(){
    Serial.println("Vive trackers started!");
    Serial.print("Connecting to ");
    Serial.print(ssid);

    WiFi.mode(WIFI_MODE_STA);
    WiFi.config(ipLocal1, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 254, 0));
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

// This function will perform all the data processing neccessary for each tick
void GTARobot::processTick()
{
    posSending();   // Send robot position, the function safe checks if it is time
    htmlServer.serve(); // Serve the webpage
    handleCanMsg();
    handleRobotMsg();

    // The overall state of the system is set by the user when he clicks
    // certain buttons on the webpage. Depending on the state, it will perform
    // a different functionality
    switch(m_system_state)
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


/* Robot functionality function */
void GTARobot::moveRobot()
{
    // This is used for manual control The linear and angular velocities
    // of the robot are determined by the robotlocomotion object
    rl.calculate_wheel_vel();
    rl.updateDirection();
    rl.updatePWM();
}

void GTARobot::moveToGivenPos(){
    // This function is used to autonomously move to a specific xy location
    // in this case we are moving to a single can that is on the field

    static int canNumToFollow;

    // Determine which can to follow
    for (canNumToFollow = 0; canNumToFollow <= 7; canNumToFollow ++){
        if (cans[canNumToFollow].id != 0){
            break;
        }
    }

    float angle_1, angle_2, diff_angle;
    float dist = sqrt(pow(cans[canNumToFollow].y - m_y, 2) + pow(cans[canNumToFollow].x - m_x, 2));
    
    // Compute a vector from the robot to the can and calculate the angle
    angle_1 = atan2(cans[canNumToFollow].y - m_y, cans[canNumToFollow].x - m_x); 

    // Compute a vector from the left vive object to the right object and calculate the angle
    angle_2 = atan2(vive2.xCoord() - vive1.xCoord(), vive2.yCoord() - vive1.yCoord()); 

    // target angle - current angle
    // We want our current angle to be 90 offset from the target angle because the front of the robot
    // is offset 90 degrees from the vector made from our left vive to the right vive
    diff_angle = (angle_2 > PI / 2 && angle_2 <= PI) ?  angle_1 - (angle_2 - 3 * PI / 2) : angle_1 - (angle_2 + PI / 2);

    if (dist > 700){
        if (fabs(diff_angle) > 10 * PI / 180){
            if (diff_angle < 0){
                    rl.turnRight(0, 0.15);
            }
            else if (diff_angle > 0){
                rl.turnLeft(0, 0.15);
            }
            angle_1 = atan2(cans[canNumToFollow].y - m_y, cans[canNumToFollow].x - m_x);       // angle from car to can
            angle_2 = atan2(vive2.xCoord() - vive1.xCoord(), vive2.yCoord() - vive1.yCoord()); 
            diff_angle = (angle_2 >  PI / 2 && angle_2 <=  PI) ?  angle_1 - (angle_2 - 3 * PI / 2) : angle_1 - (angle_2 + PI / 2);
            Serial.printf("Robo x: %d, y: %d\t", m_x, m_y);
            Serial.printf("Can x: %d, y: %d \t", cans[canNumToFollow].x, cans[canNumToFollow].y);
            Serial.printf("angle1: %f degree\t angle2: %f degree\t", angle_1 * 180 / PI, angle_2 * 180 / PI);
            Serial.printf("Diff angle: %f degree\n", diff_angle * 180 / PI);
        }
        rl.goStraight(0.4);
    }
    else{
        rl.vel.lin_vel = 0;
        rl.vel.anglr_vel = 0; 
    }
}

void  GTARobot::wallFollowing(){
    // Only using the right and front sensor to perform wall following in the counter clockwise direction
    float right_dist = right_ultrasonic.getDistance();    
    float front_dist = front_ultrasonic.getDistance();    

    // Only used for debugging
    #if SERIAL_LOGGING
    left_dist != HCSR04_OUT_OF_RANGE ? Serial.printf("Left: %f\n", left_dist) : Serial.println("Left: Out of Range");
    front_dist != HCSR04_OUT_OF_RANGE ? Serial.printf("Front: %f\n", front_dist) : Serial.println("Front: Out of Range");
    right_dist != HCSR04_OUT_OF_RANGE ? Serial.printf("Right: %f\n", right_dist) : Serial.println("Right: Out of Range");
    #endif

    // The turn is based on the time that it started turning and we wait for a delta time
    static uint32_t time_started_turning;

    switch (m_action_state)
    {
    case IDLE:
        // This state is just to ensure that we always start at the same point
        m_action_state = ADVANCING;
        break;
    case ADVANCING:
        // keep advancing until there is a wall in front, if the sensor is to close, it will have
        // and HCSR04_OUT_OF_RANGE value
        if( !(front_dist <= 20 || front_dist == HCSR04_OUT_OF_RANGE))
        {
            // If the robot is too close to the wall then it keeps moving forward but slowly
            // drifts to the left. If it is too far from the wall then it slowly drifts to
            // the right. We have a certain margin so that the bang bang controller is not too
            // jiterry; this create a type of hysterisis. If the robot is whithin the margin
            // then it goes straight
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
            m_action_state = TURNING;
            time_started_turning = millis();
        }
        break;
    case TURNING:
        // Keep turning left until enought time has elapsed
        if ( millis() - time_started_turning <= 300)
        {
            rl.turnLeft(0.0, 0.2);
        }
        else
        {
            m_action_state = ADVANCING;
        }
        break;   
    }
}

void GTARobot::beaconSensing()
{
    #if BEACON_JUST_READ_FREQUENCY

    m_r_beacon.verifyFrequency();
    m_l_beacon.verifyFrequency();

    int l_freq = m_l_beacon.getFrequency();
    int r_freq = m_r_beacon.getFrequency();
    Serial.printf("Left: %d\tRight: %d\n", l_freq, r_freq);
    #else

    /*  This is a hysterisis-based algorithm that doesn't determine the presence of the beacon
    *   based directly on whether the read frequency is approximately the expected frequency or
    *   not. The algorithm has three parameters that can be tuned: BEACON_TOP_THRESHOLD,
    *   BEACON_BOTTOM_THRESHOLD, and BEACON_MAX_PERSISTANCE. A beacon detector will only be
    *   certain that the beacon is wihin its line of sight if it has detected the beacon for a
    *   specific amount of times; this quantity is defined by BEACON_TOP_THRESHOLD. Then it will
    *   only determine that it no longer sees the beacon if the has stopped seeing it for a
    *   a specific amount of times (cycles); this is specified by BEACON_BOTTOM_THRESHOLD. This
    *   algorithm is run at a constant period of 10ms, so this number of times seen is updated
    *   every cycle, either adding 1 or subtracting 1 to the count. The third parameter,
    *   BEACON_MAX_PERSISTANCE is used to avoid a lag in the amount of cycles it takes for the
    *   diode to determine that it has stopped seeing the beacon, it is similar to doing intergrator
    *   anti-windup in a PID a controller.
    */
    
    // These two function calls should be made every 10ms to make sure that there
    // is still a signal, since the period is determined with the rising edge of the signal,
    // the frequency will not be updated if the signal stops coming.
    m_r_beacon.verifyFrequency();
    m_l_beacon.verifyFrequency();

    int l_freq = m_l_beacon.getFrequency();
    int r_freq = m_r_beacon.getFrequency();

    // Static variables to determine the consistancy of the read signals
    static int l_times_seen = 0;
    static int r_times_seen = 0;

    // Update assurance of seeing the beacon
    (l_freq == 23 || l_freq == 700) ? l_times_seen++ : l_times_seen--;
    (r_freq == 23 || r_freq == 700) ? r_times_seen++ : r_times_seen--;

    // Clamp limits, this is similar to doing integrator anti-windup on a PID controller
    l_times_seen = min(BEACON_MAX_PERSISTANCE, max(0, l_times_seen));
    r_times_seen = min(BEACON_MAX_PERSISTANCE, max(0, r_times_seen));

    switch (m_action_state)
    {
    case IDLE:
        // This states is just used to make sure the the functionality always starts from
        // the same point
        rl.goStraight(0);   // Stop the robot
        m_action_state = TURNING;
        break;
    case TURNING:
        // Neither left nor right diodes are sure they see the beacon
        if (l_times_seen < BEACON_TOP_THRESHOLD && r_times_seen < BEACON_TOP_THRESHOLD)
        {
            rl.turnLeft(0, 0.15);
        }
        // Left is certain it sees the beacon but right is not
        else if (l_times_seen >= BEACON_TOP_THRESHOLD && r_times_seen < BEACON_TOP_THRESHOLD)
        {
            rl.turnLeft(0, 0.15);
        }
        // Right is certain it sees the beacon but left is not
        else if (l_times_seen < BEACON_TOP_THRESHOLD && r_times_seen >= BEACON_TOP_THRESHOLD)
        {
            rl.turnRight(0, 0.15);
        }
        // Both diodes are certain that they see the beacon
        else if ( l_times_seen >= BEACON_TOP_THRESHOLD && r_times_seen >= BEACON_TOP_THRESHOLD)
        {
            m_action_state = ADVANCING;
        }
        break;
    case ADVANCING:
        // Go straight until you're not sure that you see the beacon ahead anymore
        rl.goStraight(0.2);
        if ( l_times_seen <= BEACON_BOTTOM_TRESHOLD || r_times_seen <= BEACON_BOTTOM_TRESHOLD)
        {
            m_action_state = IDLE;
        }
        break;
    }
#endif
}

/* Member functions for processing incomming and outgoing UDP messages */
void GTARobot::posSending(){
    static uint64_t previous_time;

    setRoboID(); // set robot ID

    // Even though our system tick is 10ms, we only send data every 100ms to
    // not clutter the network
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
            // calculateMovingAverage(m_x, avgPos[0], 3);
            // calculateMovingAverage(m_y, avgPos[1], 3);
            calculateMovingAverage(m_x, avgPos[1], 3);
            calculateMovingAverage(m_y, avgPos[0], 3);
            #else
            m_x = avgPos[1];
            m_y = avgPos[0];
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

        // This is used for debugging purposes in which we plot the robot's
        // xy position using Arduino's serial plotter
        #if SERIAL_PLOTTING
        Serial.print("X:");
        Serial.print(m_x);
        Serial.print("\t");
        Serial.print("Y:");
        Serial.println(m_y);
        #elif SERIAL_LOGGING
        // Serial.printf("sending data acquired by vive: %s\n", s);
        #endif

        // Only resync the vive object that is not locked on instead of both
        if (vive1.status() != VIVE_LOCKEDON)
        {
            vive1.sync(15); // try to resync 15 times (nonblocking)
        }
        if (vive2.status() != VIVE_LOCKEDON)
        {
            vive2.sync(15); // try to resync 15 times (nonblocking)
        }

        // Update static variable to know when to send data again
        previous_time = millis();
    }
}

void GTARobot::fncUdpSend(char* datastr, int len){
    robotUDPServer.beginPacket(ipTarget, robotUDPPort);
    robotUDPServer.write((uint8_t*)datastr, len);
    robotUDPServer.endPacket();
}

void GTARobot::handleCanMsg(){

    const int UDP_PACKET_SIZE = 14;
    uint8_t packetBuffer[UDP_PACKET_SIZE];

    int cb = canUDPServer.parsePacket(); // length of the packet
    if(cb){

        packetBuffer[cb] = 0; // null terminate string

        canUDPServer.read(packetBuffer, UDP_PACKET_SIZE);

        // Serial.printf("packet buffer: %s\n", packetBuffer);

        int canID = atoi((char *)packetBuffer); // 1st indexed char
        int x = atoi((char *) packetBuffer+2); // #:####,#### 2nd indexed char
        int y = atoi((char *) packetBuffer+7); // 7th indexed char

        //updateCanData(canID, x, y);
        cans[canID - 1].id = canID;
        // cans[canID - 1].x = x;
        // cans[canID - 1].y = y;
        cans[canID - 1].x = y;
        cans[canID - 1].y = x; 

        //Serial.printf("From can %d: %d, %d\n\r", canID, x, y);
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


/* Helper function */ 
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

void GTARobot::setRoboID(){
    // Use two pins to have a total of 4 configurations. Our robot ID is
    // represented a binary number composed of the state of these two
    // pins which can be either logically high or low.
    // We then shift one of the pin state one to the left to represent bit 1
    // in the composed binary number while the other pin state representes bit 0
    roboID = (digitalRead(DIP_SWITCH_PIN1) << 1) + digitalRead(DIP_SWITCH_PIN2) + 1;
}

void GTARobot::setState(robot_system_states system, robot_action_states action)
{
    m_system_state = system;
    m_action_state = action;
}


