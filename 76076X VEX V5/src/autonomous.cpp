#include "main.h"
#include "chassis.hpp"

extern Chassis myRobot; 

void red_close_side() {
    myRobot.drive_forward(100, true);
    pros::delay(1000);
    myRobot.stop();
}

void blue_far_side() {
    myRobot.drive_forward(100, false);
    pros::delay(1000);
    myRobot.stop();
}