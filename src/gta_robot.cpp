#include "gta_robot.h"

GTARobot::GTARobot()
{}

GTARobot::~GTARobot()
{}

// This function will perform all the data processing neccessary for each tick
void GTARobot::processTick()
{

}

void GTARobot::moveToGivenPos(){
    static int canNumToFollow;
    float slope_mypos_to_can;
    float slope_two_vive;
    float slope_multiple;

    int can_x;
    int can_y;

    // assume the line between the middle point of two vive to the can point is ax + by + c = 0, direction from middle point to can
    int a;
    int b;
    int c;
    int d;


    int my_x = *(robots + roboID - 1).x; // our own robot ID, middle point
    int my_y = *(robots + roboID - 1).y;

    // determine which can to follow
    for (canNumToFollow = 0; canNumToFollow <= 7; canNumToFollow ++){
        if (*(cans + canNumToFollow).id != 0){
            can_x = *(cans + canNumToFollow).x;
            can_y = *(cans + canNumToFollow).y;
            break;
        }
    }

    slope_mypos_to_can = (float)((my_y - *(cans + canNumToFollow).y) / (my_x - *(cans + canNumToFollow).x));
    slope_two_vive = (float)(y1 - y2) / (x1 - x2);
    slope_multiple = slope_two_vive * slope_mypos_to_can;

    a = can_y - my_y;
    b = my_x - can_x;
    c = can_x * my_y - my_x * can_y;
    d = a * x1 + b * y1 + c;

    // margin remains to tune
    if (abs(slope_multiple - (-1)) <= 0.05 && d < 0)
    {
        goStraight(0.5);
    }
    else {
        rl.turnLeft();
    }
}