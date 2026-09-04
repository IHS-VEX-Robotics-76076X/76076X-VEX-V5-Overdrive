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

// These all live in main.cpp - this is how we reach them from here.
extern Chassis myRobot;
extern pros::MotorGroup cascade_motors;
extern pros::Motor intake_motor;
extern pros::Motor arm_motor;
extern std::atomic<bool> show_status; // toggled by the LCD center button (see main.cpp)

// Shows robot battery %, controller connection, and any drivetrain/mechanism
// motor faults on LCD line 2, when show_status is toggled on.
static void update_status_display(pros::Controller &master) {
    if (!show_status) return;

    // A non-zero fault value means a motor is overheating, drawing too much
    // current, or reporting a driver fault.
    bool fault = myRobot.has_fault()
        || intake_motor.get_faults() != 0
        || arm_motor.get_faults() != 0;

    for (auto flags : cascade_motors.get_faults_all()) {
        if (flags != 0) { fault = true; break; }
    }

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

        // cascade lift: L1 up / L2 down. Holding both cancels out to 0, which
        // is what we want. Both lift motors move together as one group.
        int cascadeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) cascadeSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) cascadeSpeed -= 127;
        cascade_motors.move(cascadeSpeed);

        // arm: R1 up / R2 down.
        int armSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) armSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) armSpeed -= 127;
        arm_motor.move(armSpeed);

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
