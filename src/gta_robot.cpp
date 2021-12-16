#include "gta_robot.h"

GTARobot::GTARobot()
{
    Serial.begin(115200);
    Serial.println("Vive trackers started!");

    Serial.print("Connecting to "); Serial.print(ssid);
    WiFi.config(ipLocal1, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
        
    WiFi.begin(ssid, password);

    robotUDPServer.begin(robotUDPPort);

    vive1.begin();
    /* comment it out if you only use one vive circuit */
    vive2.begin();
}

GTARobot::~GTARobot()
{}

// This function will perform all the data processing neccessary for each tick
void GTARobot::processTick()
{

}

void GTARobot::viveUDPSetup(){
    Serial.begin(115200);
    Serial.println("Vive trackers started!");

    Serial.print("Connecting to "); Serial.print(ssid);
    WiFi.config(ipLocal1, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
        
    WiFi.begin(ssid, password);

    robotUDPServer.begin(robotUDPPort); // robot UDP port 2510
    canUDPServer.begin(canUDPPort); // can UDP port 1510

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

    if (millis() - previous_time >= 100){
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
        sprintf(s, "%1d:%4d,%4d", 1, *avgPos, *(avgPos + 1)); // robot id can be changed
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

        int x = atoi((char *) packetBuffer+2); // #:####,#### 2nd indexed char
        int y = atoi((char *) packetBuffer+7); // 7th indexed char
        int canID = atoi((char *)packetBuffer); // 1st indexed char

        //updateCanData(canID, x, y);
        cans[canID - 1].id = canID;
        cans[canID - 1].x = x;
        cans[canID - 1].y = y;

        Serial.printf("From can %d: %d, %d\n\r", canID, x, y);
    }
}


void GTARobot::handleRoboMsg(){
    const int UDP_PACKET_SIZE = 14;
    uint8_t packetBuffer[UDP_PACKET_SIZE]; 

    int cb = robotUDPServer.parsePacket();  // length of the packet
    if(cb){
        packetBuffer[cb] = 0;  // null terminate string

        robotUDPServer.read(packetBuffer, UDP_PACKET_SIZE);

        int x = atoi((char *) packetBuffer+2); // #:####,#### 2nd indexed char
        int y = atoi((char *) packetBuffer+7); // 7th indexed char
        int robotID = atoi((char *)packetBuffer); // 1st indexed char

        //updateRoboData(robotID, x, y);
        robots[robotID - 1].id = robotID;
        robots[robotID - 1].x = x;   
        robots[robotID - 1].y = y;

        Serial.printf("From robot %d: %d, %d\n\r", robotID, x, y);
    }
}
