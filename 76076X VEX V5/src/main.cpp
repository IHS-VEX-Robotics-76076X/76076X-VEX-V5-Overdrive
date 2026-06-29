#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"

void red_close_side();
void blue_far_side();

pros::Motor cascade_motor(CASCADE_MOTOR_PORT);
pros::Motor intake_motor(INTAKE_MOTOR_PORT);
pros::Motor arm_turn_motor(ARM_TURN_MOTOR_PORT);
pros::Motor clamp_motor(CLAMP_MOTOR_PORT);
pros::Imu inertial_sensor(INERTIAL_SENSOR_PORT);

// Drive train: 3 motors per side.
Chassis myRobot(
    {-1, -2, -3},
    {4, 5, 6},
    &inertial_sensor,
    PID(DEFAULT_DRIVE_KP, DEFAULT_DRIVE_KI, DEFAULT_DRIVE_KD),
    PID(DEFAULT_TURN_KP, DEFAULT_TURN_KI, DEFAULT_TURN_KD)
); // chassis

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 * 
 * prauton goes here i think but we prolly wont need it
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "jedidiah is such an awesome programmer i think he deserves a pay");

	pros::lcd::register_btn1_cb(on_center_button);
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
void competition_initialize() {}

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
	red_close_side(); //replace with whatever side ig
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