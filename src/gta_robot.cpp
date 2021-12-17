#include "gta_robot.h"


GTARobot::GTARobot()
    : m_robo_state(MANUAL)
{
}

GTARobot::~GTARobot()
{}

// This function will perform all the data processing neccessary for each tick
void GTARobot::processTick()
{
    switch (m_robo_state)
    {
        case MANUAL:
            break;
        case AUTONOMOUS:
            break;
        case WALL_FOLLOWING:
            wallFollowing();
            break;
        case BEACON_SENSING:
            beaconSensing();
            break;
        default:
    }
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