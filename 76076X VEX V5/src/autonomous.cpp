#include "main.h"
#include "chassis.hpp"

extern Chassis myRobot;

// Override AWP (SC8): 7+ pins placed for your alliance on your own side of
// the auton line, 3+ goals with 2+ each, neither robot touching the
// perimeter, no violations. All routines below must stay on their own side
// (SG7) and end clear of the perimeter - that last 0.5s back-off is the
// easiest AWP point to throw away.
// TODO: replace these skeleton drives with measured field cycles (preload +
// closest alliance goal + nearest neutral short) once paths are tuned.

// TODO: placeholder routine (drives forward for 1 second) - replace with the
// actual red-alliance, close-side autonomous once the game strategy is set.
void red_close_side() {
    myRobot.drive_forward(100);
    pros::delay(1000);
    myRobot.stop();
}

void red_far_side() {
    myRobot.drive_forward(80);
    pros::delay(800);
    myRobot.stop();
}

void blue_close_side() {
    myRobot.drive_forward(100);
    pros::delay(1000);
    myRobot.stop();
}

// TODO: placeholder routine (drives backward for 1 second) - replace with the
// actual blue-alliance, far-side autonomous once the game strategy is set.
void blue_far_side() {
    myRobot.drive_forward(-100);
    pros::delay(1000);
    myRobot.stop();
}

void skills_auton() {
    // Skills: red loaders only, goals start empty. Fill with solo cycle.
    myRobot.drive_forward(60);
    pros::delay(800);
    myRobot.stop();
}

void safe_auton() {
    // No-move: guarantees no SG7 cross / perimeter violation when paired
    // with an alliance partner going for AWP.
    myRobot.stop();
}