/**
 * op control here please
 */

#include "main.h"
#include "util.hpp"

extern pros::Motor left_motor;
extern pros::Motor right_motor;

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        int dir  = util::deadband(master.get_analog(ANALOG_LEFT_Y));   // forward/back
        int turn = util::deadband(master.get_analog(ANALOG_RIGHT_X));  // :P

        left_motor.move(dir - turn);   // :3
        right_motor.move(dir + turn);

        pros::delay(20);
    }
}