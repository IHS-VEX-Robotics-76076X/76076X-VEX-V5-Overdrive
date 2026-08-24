// ===========================================================================
//                          AUTONOMOUS ROUTINES
// ===========================================================================
//
// These run during the 15 second autonomous period, with nobody driving.
// Which one runs is chosen with the LCD left/right buttons before the match
// starts (see competition_initialize() in main.cpp).
//
// Useful things you can call here, all from myRobot:
//
//   drive_distance(inches)          drive straight, negative goes backward
//   turn_degrees(degrees)           turn in place, positive turns right
//   swing_turn(degrees, side)       turn around one locked wheel
//   drive_to_point(x, y)            drive to a spot on the field
//   follow_path({{x1,y1},{x2,y2}})  drive through several spots in order
//   reset_position(x, y, heading)   declare where the robot is starting
//
// Every one of these blocks until it finishes, so they run in order, one
// after another. To use anything that involves field position, call
// reset_position() first so the robot knows where it is starting from.
//
// The mechanism motors (intake_motor, arm_motor, cascade_motors) are
// declared in main.cpp. To use one here, add an `extern` line for it below,
// the same way myRobot is declared.
//
// ===========================================================================

#include "main.h"
#include "chassis.hpp"

extern Chassis myRobot;

// TODO: replace with the real red-alliance, close-side routine once the
// game strategy is decided. Right now it just drives forward for 1 second.
void red_close_side() {
    myRobot.drive_forward(100, true);
    pros::delay(1000);
    myRobot.stop();
}

// TODO: replace with the real blue-alliance, far-side routine once the game
// strategy is decided. Right now it just drives backward for 1 second.
void blue_far_side() {
    myRobot.drive_forward(100, false);
    pros::delay(1000);
    myRobot.stop();
}
