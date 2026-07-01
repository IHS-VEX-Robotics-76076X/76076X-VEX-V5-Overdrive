#include "main.h"
#include "chassis.hpp"

extern Chassis myRobot; 

// TODO: placeholder routine (drives forward for 1 second) - replace with the
// actual red-alliance, close-side autonomous once the game strategy is set.
void red_close_side() {
    myRobot.drive_forward(100, true);
    pros::delay(1000);
    myRobot.stop();
}

// TODO: placeholder routine (drives backward for 1 second) - replace with the
// actual blue-alliance, far-side autonomous once the game strategy is set.
void blue_far_side() {
    myRobot.drive_forward(100, false);
    pros::delay(1000);
    myRobot.stop();
}