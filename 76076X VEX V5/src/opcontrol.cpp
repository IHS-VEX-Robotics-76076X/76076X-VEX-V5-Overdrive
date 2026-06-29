/**
 * op control here please
 */

#include "main.h"
#include "chassis.hpp"
#include "util.hpp"

extern Chassis myRobot;
extern pros::Motor cascade_motor;
extern pros::Motor intake_motor;
extern pros::Motor arm_turn_motor;
extern pros::Motor clamp_motor;

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        int forward = util::deadband(static_cast<int>(master.get_analog(ANALOG_AXIS_3)));
        int strafe = util::deadband(static_cast<int>(master.get_analog(ANALOG_AXIS_1)));

        myRobot.drive(forward + strafe, forward - strafe);

        int cascadeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) cascadeSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) cascadeSpeed -= 127;
        cascade_motor.move(cascadeSpeed);

        int armSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) armSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) armSpeed -= 127;
        arm_turn_motor.move(armSpeed);

        clamp_motor.move(master.get_digital(E_CONTROLLER_DIGITAL_A) ? 127 : 0);
        intake_motor.move(master.get_digital(E_CONTROLLER_DIGITAL_X) ? 127 : 0);

        pros::delay(20);
    }
}