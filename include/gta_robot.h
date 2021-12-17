#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

# include "RobotLocomotion.h"

typedef struct Robot_data{
  int id = 0;
  int x = 0; 
  int y = 0;
} robot;

typedef struct Can_data{
  int id;
  int x;
  int y;
} can;

robot robots[4];
can cans[8];

class GTARobot
{
public:
    // 2 vive objects
    // 3 ultrasonic sensor objects
    // 2 motor objects
    // 1 gripper object
    // 1 beacon detector object
    RobotLocomotion rl = RobotLocomotion();

    GTARobot();
    ~GTARobot();
    void processTick();

    // functionality:
    void moveToGivenPos();
};

#endif /* GTA_ROBOT */