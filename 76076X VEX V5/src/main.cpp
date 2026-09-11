#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"

#include <vector>
#include <atomic>

void red_close_side();
void red_far_side();
void blue_close_side();
void blue_far_side();
void skills_auton();
void safe_auton();

// Autonomous selector: cycled with the LCD left/right buttons in
// competition_initialize() below. Override AWP needs 7 pins on 3 goals on
// your own side + both robots off the perimeter - one routine per
// color/side plus skills/safe is the minimum useful set.
enum class AutonRoutine {
	RED_CLOSE,
	RED_FAR,
	BLUE_CLOSE,
	BLUE_FAR,
	SKILLS,
	SAFE
};

static AutonRoutine selected_auton = AutonRoutine::RED_CLOSE;

static const char* auton_name(AutonRoutine r) {
	switch (r) {
		case AutonRoutine::RED_CLOSE:  return "Auton: RED CLOSE";
		case AutonRoutine::RED_FAR:    return "Auton: RED FAR";
		case AutonRoutine::BLUE_CLOSE: return "Auton: BLUE CLOSE";
		case AutonRoutine::BLUE_FAR:   return "Auton: BLUE FAR";
		case AutonRoutine::SKILLS:     return "Auton: SKILLS";
		case AutonRoutine::SAFE:       return "Auton: SAFE (no move)";
	}
	return "Auton: ?";
}

// ---------------------------------------------------------------------------
// HARDWARE
//
// Every motor and sensor is created here, once, and shared with the rest of
// the code. All port numbers and cartridge colors come from config.hpp -
// change them THERE, not here.
// ---------------------------------------------------------------------------

// Cascade lift: two motors that always move together, so they're one group.
pros::MotorGroup cascade_motors(
    std::vector<std::int8_t>(CASCADE_MOTOR_PORTS.begin(), CASCADE_MOTOR_PORTS.end()),
    CASCADE_MOTOR_GEARSET);

// Intake.
pros::Motor intake_motor(INTAKE_MOTOR_PORT, INTAKE_MOTOR_GEARSET);

// Sensors.
pros::Imu inertial_sensor(INERTIAL_SENSOR_PORT);    // which way we're facing
pros::Rotation tracking_wheel(TRACKING_WHEEL_PORT); // how far we've travelled

// The drivetrain: 4 motors, 2 per side, plus the IMU and both PID tunings.
//
// The IMU is handed over as nullptr when HAS_INERTIAL_SENSOR is false. That
// is what makes the chassis behave correctly without one: turn_degrees() and
// swing_turn() stop the motors and return, and start_odometry() doesn't spawn
// a task that would spin forever reading a sensor that isn't there.
Chassis myRobot(
    std::vector<std::int8_t>(LEFT_DRIVE_PORTS.begin(), LEFT_DRIVE_PORTS.end()),
    std::vector<std::int8_t>(RIGHT_DRIVE_PORTS.begin(), RIGHT_DRIVE_PORTS.end()),
    HAS_INERTIAL_SENSOR ? &inertial_sensor : nullptr,
    PID(DEFAULT_DRIVE_KP, DEFAULT_DRIVE_KI, DEFAULT_DRIVE_KD,
        DEFAULT_DRIVE_INTEGRAL_CAP, DEFAULT_DRIVE_SETTLE_ERROR, DEFAULT_DRIVE_SETTLE_VELOCITY),
    PID(DEFAULT_TURN_KP, DEFAULT_TURN_KI, DEFAULT_TURN_KD,
        DEFAULT_TURN_INTEGRAL_CAP, DEFAULT_TURN_SETTLE_ERROR, DEFAULT_TURN_SETTLE_VELOCITY),
    DEFAULT_HEADING_KP,
    DRIVE_MOTOR_GEARSET
);

// Toggled by the LCD center button (its own PROS task) and read every
// opcontrol() loop iteration (a different task) - plain bool would be an
// unsynchronized cross-task race, so this needs to be atomic even though
// both sides only ever do a simple load/store.
std::atomic<bool> show_status{false};

/**
 * A callback function for LLEMU's center button.
 *
 * Toggles the live battery/controller-connection/motor-fault status readout
 * on LCD line 2 (see update_status_display() in opcontrol.cpp).
 */
void on_center_button() {
	show_status = !show_status;
	if (!show_status) pros::lcd::clear_line(2);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "76076X Overdrive");

	// With no competition switch or field control plugged in, the brain runs
	// initialize() and then goes straight into opcontrol(). autonomous() and
	// competition_initialize() are never called at all. That makes this
	// function the only thing standing between hitting Run and driving, so
	// anything slow in here is dead time you have to sit through.
	// Two separate checks here. HAS_INERTIAL_SENSOR is what config.hpp SAYS
	// we have; is_installed() is what's ACTUALLY plugged in right now. Only
	// calibrate when both agree, so a loose cable at boot can't stall startup
	// for 3 seconds waiting on a sensor that isn't answering.
	if (HAS_INERTIAL_SENSOR && inertial_sensor.is_installed()) {
		pros::lcd::set_text(1, "Calibrating IMU (~2s)...");

		// Blocks until calibration finishes (3s timeout). The IMU's
		// get_rotation() is meaningless until then, and turning plus position
		// tracking both read it, so it's worth waiting for.
		inertial_sensor.reset(true);
		pros::lcd::set_text(1, "READY - driver control");
	} else {
		// No IMU - either not fitted yet (config.hpp) or not plugged in. Skip
		// calibration rather than stalling on it. Driving still works.
		// Turning and position tracking do not, and they no-op safely rather
		// than misbehaving.
		pros::lcd::set_text(1, "READY - no IMU, drive only");
	}

	pros::lcd::register_btn1_cb(on_center_button);

	// The lift must hold position against gravity when stopped, so it uses
	// HOLD - that's what lets you pause it partway up and have it stay there
	// instead of sinking. The intake has nothing to hold up and just needs
	// to stop cleanly, so BRAKE is enough.
	cascade_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	// Hand the tracking wheel over, THEN start position tracking. Order
	// matters: if start_odometry() runs first, odometry comes up using the
	// drive motor encoders instead of the tracking wheel.
	//
	// Both are gated on config.hpp. With no tracking wheel, odometry uses the
	// drive encoders. With no IMU, start_odometry() is a no-op anyway (the
	// chassis was built with a nullptr IMU), so this is belt-and-braces.
	if (HAS_TRACKING_WHEEL) {
		myRobot.set_tracking_wheel(&tracking_wheel, TRACKING_WHEEL_DIAMETER_INCH);
	}
	if (HAS_INERTIAL_SENSOR) {
		myRobot.start_odometry();
	}

	// Report the auton state here too, not just in competition_initialize().
	// That function only runs when field control is attached, so on a bare
	// brain this is the only place it would ever get shown.
	pros::lcd::set_text(3, AUTON_ENABLED ? "Auton: ENABLED" : "Auton: DISABLED (config.hpp)");

	util::fun(); // seeds the random number generator
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	auto step = [](int dir) {
		int v = static_cast<int>(selected_auton) + dir;
		const int N = static_cast<int>(AutonRoutine::SAFE) + 1;
		if (v < 0) v = N - 1;
		if (v >= N) v = 0;
		selected_auton = static_cast<AutonRoutine>(v);
	};
	while (true) {
		auto buttons = pros::lcd::read_buttons();
		// edge-triggered with debounce: hold-to-scroll caused skipped
		// selections when this ran every 20ms with no release wait.
		if (buttons & LCD_BTN_LEFT) {
			step(-1);
			while (pros::lcd::read_buttons() & LCD_BTN_LEFT) pros::delay(20);
		} else if (buttons & LCD_BTN_RIGHT) {
			step(1);
			while (pros::lcd::read_buttons() & LCD_BTN_RIGHT) pros::delay(20);
		}

		// Say so plainly when the selection can't do anything, rather than
		// letting someone cycle through routines that are never going to run.
		if (AUTON_ENABLED) {
			pros::lcd::set_text(3, auton_name(selected_auton));
		} else {
			pros::lcd::set_text(3, "Auton: DISABLED (config.hpp)");
		}

		pros::delay(50);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// AUTON_ENABLED is false while the robot is still being built and tested,
	// which means: sit still and do nothing for the whole autonomous period,
	// then let the driver take over. The routines in autonomous.cpp are only
	// placeholders that drive blindly forward, so running them on a real field
	// would just push the robot into whatever happens to be in front of it.
	//
	// Stopping the motors and returning is all that's needed. The 15 second
	// autonomous period is timed by the field, not by us - when it ends, the
	// field kills this task and starts opcontrol(), whether or not this
	// function has returned. Returning early just means the robot sits idle
	// for the remainder. There's nothing to count down here.
	//
	// stop() rather than an empty body, so that if anything was left moving
	// before autonomous started, it gets shut off here.
	if (!AUTON_ENABLED) {
		myRobot.stop();
		pros::lcd::set_text(3, "Auton: DISABLED (config.hpp)");
		return;
	}

	switch (selected_auton) {
		case AutonRoutine::RED_CLOSE:  red_close_side(); break;
		case AutonRoutine::RED_FAR:    red_far_side(); break;
		case AutonRoutine::BLUE_CLOSE: blue_close_side(); break;
		case AutonRoutine::BLUE_FAR:   blue_far_side();  break;
		case AutonRoutine::SKILLS:     skills_auton(); break;
		case AutonRoutine::SAFE:       safe_auton(); break;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
// `opcontrol` is implemented in src/opcontrol.cpp. Keep single definition there.