#pragma once

#include "api.h"

#include <array>
#include <cstdint>
#include <cmath>

// ===========================================================================
//                          ROBOT CONFIGURATION
// ===========================================================================
//
// This is the ONE file you edit when something about the robot changes.
// Port numbers, motor types, wheel sizes, and all the tuning numbers live
// here so you never have to go hunting through the rest of the code.
//
// The sections are:
//   1. Motor gear cartridges  (what color motors we use)
//   2. Drivetrain geometry    (wheel size, gearing, ticks per inch)
//   3. Port numbers           (what is plugged into where)
//   4. Tracking wheel         (the odometry wheel)
//   5. Driver control         (arcade vs tank)
//   6. PID tuning             (how hard the robot pushes toward a target)
//   7. Safety limits          (timeouts and acceleration limits)
//
// ===========================================================================


// ---------------------------------------------------------------------------
// 1. MOTOR GEAR CARTRIDGES
// ---------------------------------------------------------------------------
//
// Every V5 motor has a small colored plastic gear cartridge inside it. The
// color decides how fast and how strong the motor is, AND how many encoder
// "ticks" the motor counts for one full turn of its output shaft:
//
//     COLOR    SPEED      TICKS PER TURN    NOTES
//     Red      100 RPM    1800              slowest, strongest
//     Green    200 RPM     900              middle of the road
//     Blue     600 RPM     300              fastest, weakest
//
// Ticks matter because drive_distance() has to turn inches into ticks. If
// the color set here does not match the real motor, the robot will drive
// the wrong distance. TICKS_PER_REV below is worked out from this
// automatically, so the ONLY thing you change is the color.
//
// Our robot:
//   BLUE  - the 4 drivetrain motors and the intake motor (built for speed)
//   GREEN - the 2 cascade motors and the arm motor (built for lifting)

constexpr auto DRIVE_MOTOR_GEARSET   = pros::v5::MotorGears::blue;
constexpr auto INTAKE_MOTOR_GEARSET  = pros::v5::MotorGears::blue;
constexpr auto CASCADE_MOTOR_GEARSET = pros::v5::MotorGears::green;
constexpr auto ARM_MOTOR_GEARSET     = pros::v5::MotorGears::green;

// Looks up how many encoder ticks one motor revolution is, based on the
// cartridge color. Keeping this as a lookup means TICKS_PER_REV can never
// drift out of sync with DRIVE_MOTOR_GEARSET above.
constexpr double ticks_per_rev_for(pros::v5::MotorGears gearset) {
    return gearset == pros::v5::MotorGears::red   ? 1800.0   // 100 RPM
         : gearset == pros::v5::MotorGears::green ?  900.0   // 200 RPM
         :                                           300.0;  // 600 RPM (blue)
}


// ---------------------------------------------------------------------------
// 2. DRIVETRAIN GEOMETRY
// ---------------------------------------------------------------------------
//
// These three numbers are what let the code turn "drive 24 inches" into a
// number of encoder ticks to count.

// How wide the driven wheels are, measured across the middle, in inches.
constexpr double WHEEL_DIAMETER_INCH = 3.25;

// Gearing between the motor and the wheel.
//   1.0  = motor connects straight to the wheel
//   2.0  = wheel turns twice for every one motor turn (geared for speed)
//   0.5  = wheel turns half as often as the motor  (geared for torque)
constexpr double GEAR_RATIO = 1.0;

// Encoder ticks per motor revolution, taken from the cartridge color above.
constexpr double TICKS_PER_REV = ticks_per_rev_for(DRIVE_MOTOR_GEARSET);

// Encoder ticks per inch the robot actually travels. Worked out once here
// at compile time instead of being recalculated in every drive function.
//
//   ticks per inch = (ticks per motor turn * gearing) / wheel circumference
constexpr double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * M_PI);


// ---------------------------------------------------------------------------
// 3. PORT NUMBERS
// ---------------------------------------------------------------------------
//
// The V5 brain has 21 ports. Each motor and sensor is plugged into one.
//
// A NEGATIVE port number means "this motor is mounted backwards, so flip
// which way it spins." On a normal robot the left and right sides face
// opposite directions, so one whole side is usually negative.
//
// Our robot has 8 motors total:
//   4 drivetrain (2 per side)
//   2 cascade lift (they always move together)
//   1 intake
//   1 arm

// Drivetrain: 2 motors on each side.
constexpr std::array<std::int8_t, 2> LEFT_DRIVE_PORTS  = {-1, -2};
constexpr std::array<std::int8_t, 2> RIGHT_DRIVE_PORTS = { 3,  4};

// Cascade lift: 2 motors driven as one unit, so they always match.
constexpr std::array<std::int8_t, 2> CASCADE_MOTOR_PORTS = {5, -6};

// Single-motor mechanisms.
constexpr std::int8_t INTAKE_MOTOR_PORT = 7;
constexpr std::int8_t ARM_MOTOR_PORT    = 8;

// Sensors.
constexpr std::uint8_t INERTIAL_SENSOR_PORT = 9;  // the IMU / gyro


// ---------------------------------------------------------------------------
// 4. TRACKING WHEEL (ODOMETRY)
// ---------------------------------------------------------------------------
//
// A tracking wheel is a small wheel that is NOT powered by any motor. It
// just rolls along the ground and reports how far it has spun.
//
// Why bother, when the drive motors already have encoders? Because a
// powered wheel can spin without the robot actually moving - it slips when
// you accelerate hard or slam into something. A free-spinning wheel does
// not slip, so it gives a much more honest measure of distance traveled.
//
// We use ONE tracking wheel, mounted facing forward, which measures how far
// forward or backward the robot has traveled. Combined with the IMU telling
// us which direction we are pointed, that is everything needed to track the
// robot's position on the field.
//
// If this sensor is not plugged in, odometry automatically falls back to
// the drive motor encoders instead. Position tracking still works, it is
// just more likely to drift over a match.

constexpr std::int8_t TRACKING_WHEEL_PORT = 10;

// How wide the tracking wheel is, in inches. 2.75 is the common VEX size.
constexpr double TRACKING_WHEEL_DIAMETER_INCH = 2.75;


// ---------------------------------------------------------------------------
// 5. DRIVER CONTROL
// ---------------------------------------------------------------------------
//
// How the joysticks map to the drivetrain during driver control.
//
//   ARCADE - one stick does everything. Push the left stick forward to
//            drive forward, push it sideways to turn.
//   TANK   - one stick per side. Left stick controls the left wheels,
//            right stick controls the right wheels.
//
// This is a tank drivetrain (wheels only spin forward and backward), so
// "arcade" here still means turning. The robot cannot slide sideways.

enum class DriveMode { ARCADE, TANK };
constexpr DriveMode DEFAULT_DRIVE_MODE = DriveMode::ARCADE;


// ---------------------------------------------------------------------------
// 6. PID TUNING
// ---------------------------------------------------------------------------
//
// PID is what lets the robot drive an exact distance or turn an exact
// angle, instead of just guessing at motor power and hoping.
//
// The three gains, in plain terms:
//   kP - how hard to push based on how far away the target is.
//        Bigger = snappier, but too big makes the robot overshoot and
//        wobble back and forth.
//   kI - slowly builds up extra push if the robot stalls just short of the
//        target. Keep this very small, or zero.
//   kD - pushes back against fast movement to damp out overshoot. Helps
//        the robot settle instead of oscillating.
//
// There are two completely separate sets of gains because driving and
// turning work on totally different scales. Drive error is measured in
// encoder ticks and can be in the thousands. Turn error is measured in
// degrees and is never more than 180. The same numbers cannot work for
// both.
//
// All of these are starting guesses. They MUST be tuned on the real robot.

// Driving straight.
constexpr double DEFAULT_DRIVE_KP = 0.5;
constexpr double DEFAULT_DRIVE_KI = 0.0;
constexpr double DEFAULT_DRIVE_KD = 0.0;

// Turning in place.
constexpr double DEFAULT_TURN_KP = 1.0;
constexpr double DEFAULT_TURN_KI = 0.0;
constexpr double DEFAULT_TURN_KD = 0.0;

// How close is "close enough" to count as having arrived.
//
//   INTEGRAL_CAP    - ceiling on the kI buildup, so it cannot run away
//   SETTLE_ERROR    - how near the target we must be
//   SETTLE_VELOCITY - how slow we must be moving
//
// BOTH the error and the speed have to be small before the robot calls
// itself "arrived". Checking distance alone would let a robot flying past
// the target at full speed count as finished.
constexpr double DEFAULT_DRIVE_INTEGRAL_CAP = 5000.0;
constexpr double DEFAULT_DRIVE_SETTLE_ERROR = 30.0;    // ticks
constexpr double DEFAULT_DRIVE_SETTLE_VELOCITY = 5.0;  // ticks per loop

constexpr double DEFAULT_TURN_INTEGRAL_CAP = 20.0;
constexpr double DEFAULT_TURN_SETTLE_ERROR = 2.0;      // degrees
constexpr double DEFAULT_TURN_SETTLE_VELOCITY = 0.5;   // degrees per loop

// While driving straight, the IMU watches for the robot drifting off its
// starting heading and nudges one side harder to correct it. This is how
// hard it nudges. 0 turns the correction off entirely.
constexpr double DEFAULT_HEADING_KP = 1.0;


// ---------------------------------------------------------------------------
// 7. SAFETY LIMITS
// ---------------------------------------------------------------------------

// If a drive or turn never reaches its target - a jammed mechanism, a
// stalled motor, a bad PID gain - it gives up after this long instead of
// freezing the robot for the rest of the match.
constexpr int DRIVE_TIMEOUT_MS = 3000;
constexpr int TURN_TIMEOUT_MS = 2000;

// Caps how much drive power can change per loop (out of the -127 to 127
// range). Without this the robot slams from a dead stop to full power and
// the wheels just spin in place. With it, power ramps up smoothly.
constexpr double DRIVE_MAX_ACCEL_PER_LOOP = 15.0;
