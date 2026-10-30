#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"
#include "menu.hpp"

#include <vector>
#include <atomic>
#include <string>

void red_close_side();
void red_far_side();
void blue_close_side();
void blue_far_side();
void skills_auton();
void safe_auton();

// AutonRoutine, Screen and the button transitions all live in menu.hpp so
// the menu logic can be tested without a robot. This file holds the parts
// that genuinely need hardware: the button callbacks, the drawing, and
// actually calling the routines.

// ---------------------------------------------------------------------------
// DO NOT printf() FROM A GLOBAL CONSTRUCTOR
//
// Startup diagnostics that print from a global (a struct whose constructor
// calls printf, declared at file scope) have been added here twice now, and
// both times the brain came up with a completely blank screen.
//
// Global constructors run BEFORE the PROS kernel starts. printf writes
// through PROS's serial driver, which needs the RTOS scheduler running, so
// calling it that early can hang or fault. When it does, initialize() never
// runs, lcd::initialize() never runs, and the screen stays black - which
// reads as "the program isn't running" rather than "the diagnostic broke
// it", so the diagnostic gets kept and the real symptom gets chased.
//
// To trace startup, print from INSIDE initialize() or later, where the
// kernel is up. Better still, use lcd::set_text() - you can read it without
// a USB cable attached.
// ---------------------------------------------------------------------------

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

// Pneumatics. Not motors - each is a solenoid on an ADI (three-wire) port,
// started retracted (false/LOW).
//
//   claw_piston  grips the pin
//   flip_piston  lifts the pin up
//
// One solenoid can drive several cylinders plumbed together, so "the claw"
// being more than one physical cylinder still only needs the one port here.
pros::adi::DigitalOut claw_piston(CLAW_PNEUMATIC_PORT, false);
pros::adi::DigitalOut flip_piston(CLAW_FLIP_PNEUMATIC_PORT, false);

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
// ---------------------------------------------------------------------------
// MENU STATE
//
// Button callbacks run in their own PROS task while opcontrol() draws the
// screen from a different one, so this is shared between tasks and needs a
// lock. The critical sections are a handful of field copies, so a plain
// mutex is plenty.
// ---------------------------------------------------------------------------
static MenuState menu_state;
static pros::Mutex menu_mutex;

// Snapshot for the renderer - all three fields from the same instant, so the
// screen can't show one menu's heading over another menu's options.
MenuState menu_snapshot() {
	menu_mutex.take();
	MenuState copy = menu_state;
	menu_mutex.give();
	return copy;
}

// opcontrol calls this once it has finished running a requested test route.
void menu_clear_test_request() {
	menu_mutex.take();
	menu_state.runTestNow = false;
	menu_mutex.give();
}

// The three button callbacks. Each one only moves the state along - no
// motors, no drawing. Pressing a button must never block, because this task
// also delivers the other buttons.
static void apply_button(MenuButton button) {
	menu_mutex.take();
	menu_state = menu_press(menu_state, button);
	menu_mutex.give();
}

static void on_left_button()   { apply_button(MenuButton::LEFT); }
static void on_center_button() { apply_button(MenuButton::CENTER); }
static void on_right_button()  { apply_button(MenuButton::RIGHT); }

// ---------------------------------------------------------------------------
// DEVICE STATUS
// ---------------------------------------------------------------------------

// Is a Smart Port device actually responding on this port right now?
//
// Takes the port as it appears in config.hpp, so a reversed port like -2 is
// fine - the sign only describes direction, the device lives at |port|.
bool smart_port_connected(std::int8_t port) {
	int absolute = port < 0 ? -port : port;
	return pros::Motor(static_cast<std::int8_t>(absolute)).is_installed();
}

// Builds one line like "Drive L: 1 OK  2 --" for the status screen.
std::string port_pair_status(const char* label, std::int8_t a, std::int8_t b) {
	int absA = a < 0 ? -a : a;
	int absB = b < 0 ? -b : b;
	return std::string(label) + ": "
	     + std::to_string(absA) + (smart_port_connected(a) ? " OK  " : " --  ")
	     + std::to_string(absB) + (smart_port_connected(b) ? " OK" : " --");
}

// ---------------------------------------------------------------------------
// DRAWING THE SCREEN
//
// Called every loop from whichever task is live (opcontrol during driving,
// competition_initialize before a match). One renderer for every screen, so
// the display can't disagree with the menu state.
//
// Every line is written every time, blanks included. Leaving old lines up
// when a page is shorter than the last one is how you end up with two pages
// mixed together on screen.
// ---------------------------------------------------------------------------

void render_menu() {
	MenuState state = menu_snapshot();

	switch (state.screen) {
		case Screen::HOME: {
			pros::lcd::set_text(0, "76076X Overdrive");
			pros::lcd::set_text(1, std::string("Match auton: ") + auton_name(state.selected));
			pros::lcd::set_text(2, AUTON_ENABLED ? "Auton: ENABLED"
			                                     : "Auton: OFF (config.hpp)");
			pros::lcd::set_text(3, "");
			pros::lcd::set_text(4, std::string("Battery: ")
			                       + std::to_string(static_cast<int>(pros::battery::get_capacity())) + "%");
			pros::lcd::set_text(5, "");
			pros::lcd::set_text(6, "L=pick auton  M=test");
			pros::lcd::set_text(7, "R=port check");
			break;
		}

		case Screen::AUTON_SELECT: {
			pros::lcd::set_text(0, "PICK MATCH AUTON");
			pros::lcd::set_text(1, "");
			pros::lcd::set_text(2, std::string(">> ") + auton_name(state.selected));
			pros::lcd::set_text(3, "");
			pros::lcd::set_text(4, AUTON_ENABLED ? "" : "NOTE: auton is OFF");
			pros::lcd::set_text(5, "");
			pros::lcd::set_text(6, "L=next  M=confirm");
			pros::lcd::set_text(7, "R=cancel");
			break;
		}

		case Screen::TEST_MENU: {
			pros::lcd::set_text(0, "TEST AUTON");
			pros::lcd::set_text(1, "");
			pros::lcd::set_text(2, std::string(">> ") + auton_name(state.selected));
			// Flagged while the routine is actually driving, so nobody
			// assumes the screen has frozen - the robot is moving and the
			// sticks are ignored until it finishes.
			pros::lcd::set_text(3, state.runTestNow ? "RUNNING - sticks off" : "");
			pros::lcd::set_text(4, "");
			pros::lcd::set_text(5, "");
			pros::lcd::set_text(6, "L=next  M=RUN NOW");
			pros::lcd::set_text(7, "R=back to driving");
			break;
		}

		case Screen::STATUS: {
			// What is actually responding on each port right now. Unplug a
			// motor and its number flips to "--" within a loop.
			pros::lcd::set_text(0, "PORT CHECK");
			pros::lcd::set_text(1, port_pair_status("Drive L", LEFT_DRIVE_PORTS[0], LEFT_DRIVE_PORTS[1]));
			pros::lcd::set_text(2, port_pair_status("Drive R", RIGHT_DRIVE_PORTS[0], RIGHT_DRIVE_PORTS[1]));
			pros::lcd::set_text(3, port_pair_status("Cascade", CASCADE_MOTOR_PORTS[0], CASCADE_MOTOR_PORTS[1]));
			pros::lcd::set_text(4, std::string("Intake ") + std::to_string(INTAKE_MOTOR_PORT)
			                       + (smart_port_connected(INTAKE_MOTOR_PORT) ? ": OK" : ": --"));
			pros::lcd::set_text(5, std::string("IMU ") + std::to_string(INERTIAL_SENSOR_PORT)
			                       + (inertial_sensor.is_installed() ? ": OK" : ": --")
			                       + "  Track " + std::to_string(TRACKING_WHEEL_PORT)
			                       + (tracking_wheel.is_installed() ? ": OK" : ": --"));
			// ADI ports are dumb three-wire outputs with no device ID, so
			// there is genuinely nothing to query - a solenoid that is
			// unplugged looks identical to one sitting there doing nothing.
			// Saying so beats inventing a reading that would always read OK.
			pros::lcd::set_text(6, "Pneumatics: can't detect");
			pros::lcd::set_text(7, "any button = back");
			break;
		}
	}
}

// ---------------------------------------------------------------------------
// RUNNING A ROUTINE
// ---------------------------------------------------------------------------

// Calls one autonomous routine. Shared by autonomous() and by the test menu
// so a route tested on the bench is literally the same code that runs in the
// match - no second dispatch table to drift out of step.
void run_auton_routine(AutonRoutine routine) {
	switch (routine) {
		case AutonRoutine::RED_CLOSE:  red_close_side(); break;
		case AutonRoutine::RED_FAR:    red_far_side(); break;
		case AutonRoutine::BLUE_CLOSE: blue_close_side(); break;
		case AutonRoutine::BLUE_FAR:   blue_far_side(); break;
		case AutonRoutine::SKILLS:     skills_auton(); break;
		case AutonRoutine::SAFE:       safe_auton(); break;
	}
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

	// All three buttons: left picks the match auton, centre opens the test
	// menu, right shows what's plugged in. See menu.hpp for the full map.
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);

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
	// Runs only with field control attached, between initialize() and the
	// match starting. Shows the same menu as driver control, driven by the
	// same callbacks - one set of button meanings to learn, not two.
	//
	// The previous version read the buttons itself with its own left/right
	// stepping, which meant the buttons did different things depending on
	// whether a field controller happened to be plugged in.
	while (true) {
		render_menu();
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

	run_auton_routine(menu_snapshot().selected);
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