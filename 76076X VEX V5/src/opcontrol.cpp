/**
 * op control here please
 */

#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"

#include <cstdio>

extern Chassis myRobot;
extern pros::Motor cascade_motor;
extern pros::Motor intake_motor;
extern pros::Motor arm_turn_motor;
extern pros::Motor clamp_motor;
extern bool show_status; // toggled by the LCD center button (see main.cpp)

// Shows robot battery %, controller connection, and any drivetrain/mechanism
// motor faults on LCD line 2, when show_status is toggled on.
static void update_status_display(pros::Controller &master) {
    if (!show_status) return;

    bool fault = myRobot.has_fault()
        || cascade_motor.get_faults() != 0
        || intake_motor.get_faults() != 0
        || arm_turn_motor.get_faults() != 0
        || clamp_motor.get_faults() != 0;

    char buf[40];
    std::snprintf(buf, sizeof(buf), "Bat:%d%% Ctrl:%s%s",
                  static_cast<int>(pros::battery::get_capacity()),
                  master.is_connected() ? "OK" : "LOST",
                  fault ? " FAULT!" : "");
    pros::lcd::set_text(2, buf);
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

        if constexpr (DEFAULT_DRIVE_MODE == DriveMode::ARCADE) {
            int forward = util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_Y)));
            int turn = util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_X)));
            myRobot.drive(forward + turn, forward - turn);
        } else {
            int left = util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_Y)));
            int right = util::deadband(static_cast<int>(master.get_analog(ANALOG_RIGHT_Y)));
            myRobot.drive(left, right);
        }

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

        update_status_display(master);

        pros::delay(20);
    }
}
