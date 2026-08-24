// ===========================================================================
//                            DRIVER CONTROL
// ===========================================================================
//
// This runs for the whole driver-controlled part of a match. It loops
// forever, reading the controller and moving the robot to match, about 50
// times a second.
//
// Controls:
//   Left stick        drive (see DEFAULT_DRIVE_MODE in config.hpp)
//   Right stick       drive, only in TANK mode
//   L1 / L2           cascade lift up / down
//   R1 / R2           arm up / down
//   X                 intake
//   LCD center button toggle the status readout on the brain screen
//
// ===========================================================================

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
extern std::atomic<bool> show_status; // toggled by the LCD center button

// Puts battery level, controller connection, and any motor problems on line
// 2 of the brain screen. Only runs when the center button has turned the
// readout on, so it stays out of the way the rest of the time.
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

    while (true) {
        // ---- Drivetrain -------------------------------------------------
        // deadband() ignores tiny stick readings. Joysticks rarely sit at
        // exactly zero when released, and without this the robot would
        // creep around on its own whenever nobody is touching the sticks.
        if constexpr (DEFAULT_DRIVE_MODE == DriveMode::ARCADE) {
            // One stick does everything: push forward to drive, push
            // sideways to turn. Adding the turn to one side and subtracting
            // it from the other is what makes the robot rotate.
            int forward = util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_Y)));
            int turn = util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_X)));
            myRobot.drive(forward + turn, forward - turn);
        } else {
            // One stick per side of the robot.
            int left = util::deadband(static_cast<int>(master.get_analog(ANALOG_LEFT_Y)));
            int right = util::deadband(static_cast<int>(master.get_analog(ANALOG_RIGHT_Y)));
            myRobot.drive(left, right);
        }

        // ---- Cascade lift (L1 up, L2 down) ------------------------------
        // Holding both buttons cancels out to zero, which is what we want.
        int cascadeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) cascadeSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) cascadeSpeed -= 127;
        cascade_motors.move(cascadeSpeed); // both lift motors together

        // ---- Arm (R1 up, R2 down) ---------------------------------------
        int armSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) armSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) armSpeed -= 127;
        arm_motor.move(armSpeed);

        // ---- Intake (X held) --------------------------------------------
        intake_motor.move(master.get_digital(E_CONTROLLER_DIGITAL_X) ? 127 : 0);

        update_status_display(master);

        // Wait before looping. Reading the controller faster than this gains
        // nothing and just wastes brain time other tasks could use.
        pros::delay(20);
    }
}
