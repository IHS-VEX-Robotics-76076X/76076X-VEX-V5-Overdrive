#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"

#include <vector>

void red_close_side();
void blue_far_side();

// Autonomous selector: cycled with the LCD left/right buttons in
// competition_initialize() below, since blue_far_side() otherwise has no
// way to run without recompiling and redeploying between matches.
enum class AutonRoutine {
	RED_CLOSE,
	BLUE_FAR
};

static AutonRoutine selected_auton = AutonRoutine::RED_CLOSE;

pros::Motor cascade_motor(CASCADE_MOTOR_PORT);
pros::Motor intake_motor(INTAKE_MOTOR_PORT);
pros::Motor arm_turn_motor(ARM_TURN_MOTOR_PORT);
pros::Motor clamp_motor(CLAMP_MOTOR_PORT);
pros::Imu inertial_sensor(INERTIAL_SENSOR_PORT);

// Tracking wheels for odometry - see set_tracking_wheels() below and the
// port/geometry comments in config.hpp.
pros::Rotation left_tracking_wheel(LEFT_TRACKING_WHEEL_PORT);
pros::Rotation right_tracking_wheel(RIGHT_TRACKING_WHEEL_PORT);
pros::Rotation back_tracking_wheel(BACK_TRACKING_WHEEL_PORT);

// Drive train ports come from config.hpp - update the ports there, not here.
Chassis myRobot(
    std::vector<std::int8_t>(LEFT_DRIVE_PORTS.begin(), LEFT_DRIVE_PORTS.end()),
    std::vector<std::int8_t>(RIGHT_DRIVE_PORTS.begin(), RIGHT_DRIVE_PORTS.end()),
    &inertial_sensor,
    PID(DEFAULT_DRIVE_KP, DEFAULT_DRIVE_KI, DEFAULT_DRIVE_KD,
        DEFAULT_DRIVE_INTEGRAL_CAP, DEFAULT_DRIVE_SETTLE_ERROR, DEFAULT_DRIVE_SETTLE_VELOCITY),
    PID(DEFAULT_TURN_KP, DEFAULT_TURN_KI, DEFAULT_TURN_KD,
        DEFAULT_TURN_INTEGRAL_CAP, DEFAULT_TURN_SETTLE_ERROR, DEFAULT_TURN_SETTLE_VELOCITY),
    DEFAULT_HEADING_KP
); // chassis

// Toggled by the LCD center button; opcontrol.cpp reads this to decide
// whether to show live battery/controller/fault status on LCD line 2.
bool show_status = false;

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
	pros::lcd::set_text(1, "Calibrating IMU...");

	// Blocks ~2s until calibration finishes (3s safety timeout) - the IMU's
	// get_rotation() is meaningless before this, which would otherwise throw
	// off turn_degrees()/swing_turn()/odometry if they ran too soon after
	// power-on.
	inertial_sensor.reset(true);

	pros::lcd::set_text(1, "76076X Overdrive - Ready");

	pros::lcd::register_btn1_cb(on_center_button);

	// Lift/arm/clamp mechanisms hold position against gravity when stopped;
	// the intake just needs a clean stop, not a held position.
	cascade_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	arm_turn_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	clamp_motor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	intake_motor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	// Must be called before start_odometry() - see set_tracking_wheels() doc.
	myRobot.set_tracking_wheels(&left_tracking_wheel, &right_tracking_wheel,
	                             &back_tracking_wheel, TRACKING_WHEEL_DIAMETER_INCH);
	myRobot.start_odometry();
	util::fun();
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
	while (true) {
		auto buttons = pros::lcd::read_buttons();
		if (buttons & LCD_BTN_LEFT) {
			selected_auton = AutonRoutine::RED_CLOSE;
		} else if (buttons & LCD_BTN_RIGHT) {
			selected_auton = AutonRoutine::BLUE_FAR;
		}

		pros::lcd::set_text(3, selected_auton == AutonRoutine::RED_CLOSE
		                            ? "Auton: RED CLOSE   (right > blue)"
		                            : "Auton: BLUE FAR    (< left red)");

		pros::delay(20);
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
	switch (selected_auton) {
		case AutonRoutine::RED_CLOSE: red_close_side(); break;
		case AutonRoutine::BLUE_FAR:  blue_far_side();  break;
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