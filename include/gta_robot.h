#ifndef GTA_ROBOT_H
#define GTA_ROBOT_H

class GTARobot
{
public:
    // 2 vive objects
    // 3 ultrasonic sensor objects
    // 2 motor objects
    // 1 gripper object
    // 1 beacon detector object

    GTARobot();
    ~GTARobot();
    void processTick();

};

#endif /* GTA_ROBOT */