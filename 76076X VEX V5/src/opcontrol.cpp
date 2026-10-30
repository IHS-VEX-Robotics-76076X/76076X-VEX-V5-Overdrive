/**
 * Driver control: drivetrain (arcade or tank, see DEFAULT_DRIVE_MODE in
 * config.hpp) plus the cascade/claw/flip/intake mechanisms.
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
extern pros::adi::DigitalOut claw_piston;
extern pros::adi::DigitalOut flip_piston;
extern std::atomic<bool> show_status; // toggled by the LCD center button (see main.cpp)

// Shows robot battery %, controller connection, and any drivetrain/mechanism
// motor faults on LCD line 2, when show_status is toggled on.
static void update_status_display(pros::Controller &master) {
    if (!show_status) return;

    // A non-zero fault value means a motor is overheating, drawing too much
    // current, or reporting a driver fault. Pistons aren't motors and don't
    // report faults, so they're not part of this check.
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

// Adding a DriveMode without giving it a branch below would silently fall
// into the else and drive as TANK - which is exactly the bug that shipped
// when SPLIT_ARCADE was added. This turns that into a build error.
//
// It lives out here rather than inside the if/else: a static_assert whose
// condition doesn't depend on a template parameter is evaluated even in a
// discarded `if constexpr` branch, so putting it in the else fires it on
// every build regardless of the configured mode.
static_assert(DEFAULT_DRIVE_MODE == DriveMode::SPLIT_ARCADE ||
              DEFAULT_DRIVE_MODE == DriveMode::ARCADE ||
              DEFAULT_DRIVE_MODE == DriveMode::TANK,
              "Unhandled DriveMode - add a branch for it in opcontrol()");

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
        //
        // Generic parameter because the real PROS API takes a strongly typed
        // enum here while the host mock takes an int - `auto` accepts both.
        auto axis = [&master](auto channel) {
            return util::expo(util::deadband(static_cast<int>(master.get_analog(channel))),
                              OPCONTROL_EXPO_GAIN);
        };

        int targetLeft = 0;
        int targetRight = 0;

        // Every DriveMode gets its own branch. There used to be only an
        // ARCADE test and an else, so SPLIT_ARCADE - the configured default -
        // fell through to the else and quietly drove as TANK. It compiled
        // clean and no test covered it; the only symptom was the sticks
        // behaving wrong. The static_assert at the bottom means adding a
        // fourth mode without handling it is now a build error rather than
        // another silent fallthrough.
        if constexpr (DEFAULT_DRIVE_MODE == DriveMode::SPLIT_ARCADE) {
            // Drive with the LEFT stick, steer with the RIGHT stick.
            //
            // Adding the turn to one side and subtracting it from the other
            // is what rotates the robot: pushing the right stick left makes
            // the left wheels slower (or reverse) and the right wheels
            // faster, so the robot swings left.
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
        // Ramp the power up gradually so a tall stack doesn't tip when the
        // base jerks to full voltage - but let it come DOWN instantly, so
        // releasing the stick or slamming into reverse stops the robot now
        // rather than coasting through a ramp. util::slew limits both
        // directions; util::accelLimit limits only speeding up, which is
        // what OPCONTROL_SLEW_PER_LOOP's comment already describes.
        prevLeft = util::accelLimit(prevLeft, targetLeft, OPCONTROL_SLEW_PER_LOOP);
        prevRight = util::accelLimit(prevRight, targetRight, OPCONTROL_SLEW_PER_LOOP);
        myRobot.drive(prevLeft, prevRight);

        // cascade lift: R1 up / R2 down. Holding both cancels out to 0, which
        // is what we want. Both lift motors move together as one group.
        int cascadeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) cascadeSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) cascadeSpeed -= 127;
        cascade_motors.move(cascadeSpeed);        

        // intake: X in / B out (reverse clears jams and fixes cup orientation
        // for SC3 - opaque vs transparent matters for yellow scoring).
        int intakeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_X)) intakeSpeed = 127;
        else if (master.get_digital(E_CONTROLLER_DIGITAL_B)) intakeSpeed = -127;
        intake_motor.move(intakeSpeed);

        update_status_display(master);

        // claw: A toggles open/closed. new_press (not get_digital) so holding
        // the button doesn't flip it back and forth every 20ms loop.
        static bool clawOpen = false;
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
            clawOpen = !clawOpen;
            claw_piston.set_value(clawOpen);
        }

        // Y toggles the flip piston, same edge-triggered reasoning as above.
        static bool flipOut = false;
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
            flipOut = !flipOut;
            flip_piston.set_value(flipOut);
        }

        pros::delay(20);
    }
}
