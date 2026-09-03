/**
 * Driver control: drivetrain (arcade or tank, see DEFAULT_DRIVE_MODE in
 * config.hpp) plus the cascade/arm/clamp/intake mechanisms.
 */

#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"

#include <cstdio>
#include <atomic>

extern Chassis myRobot;
extern pros::Motor cascade_motor;
extern pros::Motor intake_motor;
extern pros::Motor arm_turn_motor;
extern pros::Motor clamp_motor;
extern std::atomic<bool> show_status; // toggled by the LCD center button (see main.cpp)

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
    int prevLeft = 0;
    int prevRight = 0;

    while (true) {
        int targetLeft = 0;
        int targetRight = 0;
        if constexpr (DEFAULT_DRIVE_MODE == DriveMode::ARCADE) {
            int forward = util::expo(util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_Y))), OPCONTROL_EXPO_GAIN);
            int turn = util::expo(util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_X))), OPCONTROL_EXPO_GAIN);
            targetLeft = forward + turn;
            targetRight = forward - turn;
        } else {
            targetLeft = util::expo(util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_Y))), OPCONTROL_EXPO_GAIN);
            targetRight = util::expo(util::deadband(static_cast<int>(master.get_analog(ANALOG_RIGHT_Y))), OPCONTROL_EXPO_GAIN);
        }
        // slew: tall Override stacks tip if the base jerks at full voltage
        prevLeft = util::slew(prevLeft, targetLeft, OPCONTROL_SLEW_PER_LOOP);
        prevRight = util::slew(prevRight, targetRight, OPCONTROL_SLEW_PER_LOOP);
        myRobot.drive(prevLeft, prevRight);

        int cascadeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) cascadeSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) cascadeSpeed -= 127;
        cascade_motor.move(cascadeSpeed);

        int armSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) armSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) armSpeed -= 127;
        arm_turn_motor.move(armSpeed);

        // clamp: momentary both ways (A close / Y open), else hold position.
        // Previous code only closed (127/0) with no release path.
        int clampSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_A)) clampSpeed = 127;
        else if (master.get_digital(E_CONTROLLER_DIGITAL_Y)) clampSpeed = -127;
        clamp_motor.move(clampSpeed);

        // intake: X in / B out (reverse clears jams and fixes cup orientation
        // for SC3 - opaque vs transparent matters for yellow scoring).
        int intakeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_X)) intakeSpeed = 127;
        else if (master.get_digital(E_CONTROLLER_DIGITAL_B)) intakeSpeed = -127;
        intake_motor.move(intakeSpeed);

        update_status_display(master);

        pros::delay(20);
    }
}
