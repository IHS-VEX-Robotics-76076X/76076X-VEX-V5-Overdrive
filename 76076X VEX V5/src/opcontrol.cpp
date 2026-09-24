/**
 * Driver control: drivetrain (see DEFAULT_DRIVE_MODE in config.hpp) plus the
 * cascade lift and intake.
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
extern std::atomic<bool> show_status; // toggled by the LCD center button (see main.cpp)

// Shows robot battery %, controller connection, and any drivetrain/mechanism
// motor faults on LCD line 2, when show_status is toggled on.
static void update_status_display(pros::Controller &master) {
    if (!show_status) return;

    // A non-zero fault value means a motor is overheating, drawing too much
    // current, or reporting a driver fault.
    bool fault = myRobot.has_fault()
        || intake_motor.get_faults() != 0;

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

// ---------------------------------------------------------------------------
// MECHANISMS
//
// One function per mechanism, each called once per loop. Keeping them
// separate means you can retune or rebind one without reading past the
// other, and autonomous can call the same function rather than duplicating
// the motor commands.
// ---------------------------------------------------------------------------

// Cascade lift: L1 raises, L2 lowers. Let go and it holds where it is.
//
// TWO motors, one on each side of the lift, driven as a single group so they
// physically cannot get out of sync. One of the two ports is negated in
// config.hpp because the motors face opposite directions - without that they
// would push against each other and stall instead of lifting.
//
// It's one continuous mechanism from bottom to top: hold L1 and it keeps
// rising, release and it stops. Because the brake mode is HOLD (set in
// initialize()), "stops" means it actively holds that height rather than
// sagging back down under its own weight - so you can pause it anywhere.
//
// Holding both buttons cancels to 0, which is the sane result.
void run_cascade(pros::Controller &master) {
    int speed = 0;
    if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) speed += 127;
    if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) speed -= 127;
    cascade_motors.move(speed);
}

// Intake: R1 pulls in, R2 spits out.
//
// Mirrors the lift on the other hand - L1/L2 for the cascade, R1/R2 for the
// intake - so both mechanisms live on the shoulder buttons and the driver's
// thumbs stay on the sticks.
//
// Reverse matters as much as forward - it's how you clear a jam without
// having to stop and dig something out by hand mid-match.
void run_intake(pros::Controller &master) {
    int speed = 0;
    if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) speed = 127;
    else if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) speed = -127;
    intake_motor.move(speed);
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    int prevLeft = 0;
    int prevRight = 0;

    while (true) {
        // Reads one joystick axis and cleans it up:
        //   deadband - a released stick rarely sits at exactly 0, so ignore
        //              tiny readings or the robot creeps on its own
        //   expo     - softens the middle of the stick travel for fine
        //              control, while full push still gives full power
        // Generic parameter because the real PROS API takes a strongly typed
        // enum here while the host mock takes an int - `auto` accepts both.
        auto axis = [&master](auto channel) {
            return util::expo(util::deadband(static_cast<int>(master.get_analog(channel))),
                              OPCONTROL_EXPO_GAIN);
        };

        int targetLeft = 0;
        int targetRight = 0;

        if constexpr (DEFAULT_DRIVE_MODE == DriveMode::SPLIT_ARCADE) {
            // Drive with the LEFT stick, steer with the RIGHT stick.
            //
            // Adding the turn to one side and subtracting it from the other is
            // what rotates the robot: pushing the right stick left makes the
            // left wheels slower (or reverse) and the right wheels faster, so
            // the robot swings left.
            int forward = axis(ANALOG_LEFT_Y);
            int turn = axis(ANALOG_RIGHT_X);
            targetLeft = forward + turn;
            targetRight = forward - turn;
        } else if constexpr (DEFAULT_DRIVE_MODE == DriveMode::ARCADE) {
            // Same mixing, but both axes come off the left stick.
            int forward = axis(ANALOG_LEFT_Y);
            int turn = axis(ANALOG_LEFT_X);
            targetLeft = forward + turn;
            targetRight = forward - turn;
        } else {
            // Tank: one stick per side, no mixing.
            targetLeft = axis(ANALOG_LEFT_Y);
            targetRight = axis(ANALOG_RIGHT_Y);
        }
        // Clamp first: mixing can reach +/-254, and ramping down from there
        // would add dead time before the motors see any change.
        targetLeft = static_cast<int>(util::clamp(targetLeft, -127.0, 127.0));
        targetRight = static_cast<int>(util::clamp(targetRight, -127.0, 127.0));

        // Ramp up only (tall Override stacks tip if the base jerks at full
        // voltage); braking and reversing respond immediately.
        prevLeft = util::accelLimit(prevLeft, targetLeft, OPCONTROL_SLEW_PER_LOOP);
        prevRight = util::accelLimit(prevRight, targetRight, OPCONTROL_SLEW_PER_LOOP);
        myRobot.drive(prevLeft, prevRight);

        run_cascade(master);
        run_intake(master);

        update_status_display(master);

        pros::delay(20);
    }
}
