#pragma once

#include "api.h" // for pros::v5::MotorGears, used by the cartridge settings below

#include <array>
#include <cstdint>
#include <cmath>

// Driver control style. ARCADE: one stick (left Y = forward, left X = turn).
// TANK: two sticks (left Y = left side, right Y = right side). This is a tank
// drivetrain (3 motors/side, no mecanum) - "arcade" here still means single-
// stick turning, not strafing.
enum class DriveMode { ARCADE, TANK };
constexpr DriveMode DEFAULT_DRIVE_MODE = DriveMode::ARCADE;

// ---------------------------------------------------------------------------
// MOTOR GEAR CARTRIDGES
// ---------------------------------------------------------------------------
//
// Every V5 motor has a colored plastic gear cartridge inside it. The color
// sets how fast and how strong the motor is, AND how many encoder "ticks" it
// counts per revolution:
//
//     COLOR    SPEED      TICKS PER TURN    NOTES
//     Red      100 RPM    1800              slowest, strongest
//     Green    200 RPM     900              middle
//     Blue     600 RPM     300              fastest, weakest
//
// Ticks matter because drive_distance() converts inches into ticks. If the
// color here doesn't match the real motor, the robot drives the wrong
// distance - off by a clean 3x between blue and green, with nothing to
// indicate why. TICKS_PER_REV below is derived from this automatically, so the
// ONLY thing you ever change is the color.
//
// Our robot:
//   BLUE  - the 4 drivetrain motors and the intake (built for speed)
//   GREEN - the 2 cascade motors and the arm (built for lifting)
constexpr auto DRIVE_MOTOR_GEARSET   = pros::v5::MotorGears::blue;
constexpr auto INTAKE_MOTOR_GEARSET  = pros::v5::MotorGears::blue;
constexpr auto CASCADE_MOTOR_GEARSET = pros::v5::MotorGears::green;
constexpr auto ARM_MOTOR_GEARSET     = pros::v5::MotorGears::green;

// Looks up encoder ticks per motor revolution from the cartridge color, so
// TICKS_PER_REV can never drift out of sync with the cartridge above.
constexpr double ticks_per_rev_for(pros::v5::MotorGears gearset) {
    return gearset == pros::v5::MotorGears::red   ? 1800.0   // 100 RPM
         : gearset == pros::v5::MotorGears::green ?  900.0   // 200 RPM
         :                                           300.0;  // 600 RPM (blue)
}

// ---------------------------------------------------------------------------
// DRIVETRAIN GEOMETRY
// ---------------------------------------------------------------------------
//
// These are what let the code turn "drive 24 inches" into a tick count.

// Width of the driven wheels across the middle, in inches.
constexpr double WHEEL_DIAMETER_INCH = 3.25;

// Gearing between the motor and the wheel.
//   1.0 = motor connects straight to the wheel
//   2.0 = wheel turns twice per motor turn (geared for speed)
//   0.5 = wheel turns half as often as the motor (geared for torque)
constexpr double GEAR_RATIO = 1.0;

// Encoder ticks per motor revolution, from the drivetrain cartridge color.
constexpr double TICKS_PER_REV = ticks_per_rev_for(DRIVE_MOTOR_GEARSET);

// Encoder ticks per inch the robot actually travels. Worked out once here at
// compile time rather than recalculated in every drive function.
//
//   ticks per inch = (ticks per motor turn * gearing) / wheel circumference
constexpr double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * M_PI);

// Drive motor ports: 2 motors per side (4x11W = 44W baseline).
// Override R11a caps Subsystem 1 (drivetrain) at 55W and R11b bans PTO/
// differentials off drive motors - a 3-per-side (66W) drive is illegal.
// 4x11W leaves 11W of Subsystem 1 headroom and 44W of total-budget headroom
// under the 88W robot total (R10a) for lift + manipulator.
// Left side uses reversed ports for a mirrored drivetrain.
constexpr std::array<std::int8_t, 2> LEFT_DRIVE_PORTS = {-1, -2};
constexpr std::array<std::int8_t, 2> RIGHT_DRIVE_PORTS = {3, 4};

// ---------------------------------------------------------------------------
// MECHANISM PORTS
// ---------------------------------------------------------------------------
//
// 8 motors total on this robot:
//   4 drivetrain (above)
//   2 cascade lift (always move together, so they're one MotorGroup)
//   1 intake
//   1 arm

// Cascade lift: 2 motors driven as one unit so they can never fight.
// One is negated because they face opposite directions on the lift.
constexpr std::array<std::int8_t, 2> CASCADE_MOTOR_PORTS = {5, -6};

// Single-motor mechanisms.
constexpr std::int8_t INTAKE_MOTOR_PORT = 7;
constexpr std::int8_t ARM_MOTOR_PORT    = 8;

// Sensors.
constexpr std::uint8_t INERTIAL_SENSOR_PORT = 11; // the IMU / gyro

// Motor power budget (V5 Smart Motors: 11W full, 5.5W half).
constexpr int WATT_PER_11W_MOTOR = 11;
constexpr int SUBSYSTEM1_MAX_WATT = 55; // R11a drivetrain cap
constexpr int ROBOT_MAX_WATT = 88;      // R10a robot total cap
constexpr int MECHANISM_MOTOR_COUNT =
    CASCADE_MOTOR_PORTS.size() + 2; // + intake + arm
static_assert((LEFT_DRIVE_PORTS.size() + RIGHT_DRIVE_PORTS.size()) * WATT_PER_11W_MOTOR <= SUBSYSTEM1_MAX_WATT,
    "Illegal drivetrain: Subsystem 1 exceeds 55W (R11a). Use at most 5x11W, e.g. 4x11W.");
static_assert((LEFT_DRIVE_PORTS.size() + RIGHT_DRIVE_PORTS.size() + MECHANISM_MOTOR_COUNT) * WATT_PER_11W_MOTOR <= ROBOT_MAX_WATT,
    "Illegal robot: total exceeds 88W (R10a). Count drive + cascade + intake + arm.");

// ---------------------------------------------------------------------------
// TRACKING WHEEL (ODOMETRY) - we have exactly ONE, facing forward
// ---------------------------------------------------------------------------
//
// A tracking wheel is a small wheel that NO motor drives. It just rolls along
// the ground and reports how far it has spun.
//
// Why bother, when the drive motors already have encoders? Because a powered
// wheel can spin without the robot actually moving - it slips when you
// accelerate hard or shove into something. A free-spinning wheel doesn't lie.
//
// One forward-facing wheel is enough for full position tracking: the wheel
// says HOW FAR we went, the IMU says WHICH WAY we were pointed. Combine them
// every tick and you can add up the robot's path.
//
// What one wheel can't do is measure sideways drift. If the robot gets shoved
// sideways, odometry won't notice. That needs a second wheel mounted
// perpendicular, which we don't have.
//
// If this sensor isn't plugged in, odometry automatically falls back to the
// drive motor encoders. Position tracking still works, just with more drift.
//
// Port uses the same negative-for-reversed convention as motor ports. If the
// tracked distance goes DOWN when the robot drives forward, negate this.
constexpr std::int8_t TRACKING_WHEEL_PORT = 12;

// Width of the tracking wheel across the middle, in inches. 2.75 is the
// common VEX size.
constexpr double TRACKING_WHEEL_DIAMETER_INCH = 2.75;

// Default PID gains (safe defaults; tune for your robot)
constexpr double DEFAULT_DRIVE_KP = 0.5;
constexpr double DEFAULT_DRIVE_KI = 0.0;
constexpr double DEFAULT_DRIVE_KD = 0.0;

constexpr double DEFAULT_TURN_KP = 1.0;
constexpr double DEFAULT_TURN_KI = 0.0;
constexpr double DEFAULT_TURN_KD = 0.0;

// PID windup guard and settling tolerances. Drive error is in encoder ticks
// (can be thousands) and turn error is in degrees (0-180), so they need very
// different scales - do not share one set of values between the two PIDs.
constexpr double DEFAULT_DRIVE_INTEGRAL_CAP = 5000.0;
constexpr double DEFAULT_DRIVE_SETTLE_ERROR = 30.0;    // ticks
constexpr double DEFAULT_DRIVE_SETTLE_VELOCITY = 5.0;  // ticks/loop

constexpr double DEFAULT_TURN_INTEGRAL_CAP = 20.0;
constexpr double DEFAULT_TURN_SETTLE_ERROR = 2.0;      // degrees
constexpr double DEFAULT_TURN_SETTLE_VELOCITY = 0.5;   // degrees/loop

// Safety timeouts: if a movement's PID loop never settles (stall, jam,
// disconnected motor, bad gains), it bails out after this many ms instead of
// hanging the autonomous/opcontrol task forever.
constexpr int DRIVE_TIMEOUT_MS = 3000;
constexpr int TURN_TIMEOUT_MS = 2000;

// Stall detection: exit early when commanding significant power but making
// no encoder/IMU progress (e.g. pushed into a goal). Saves auton clock vs
// burning the full timeout above. Progress is measured as position delta
// (no extra sensor API needed, works on device and host mock alike).
constexpr int STALL_TIMEOUT_MS = 500;              // no progress for this long -> stalled
constexpr double STALL_DRIVE_PROGRESS_TICKS = 10.0; // min encoder ticks considered progress
constexpr double STALL_TURN_PROGRESS_DEG = 1.0;     // min degrees considered progress
constexpr double STALL_MIN_OUTPUT = 20.0;           // ignore stall when barely commanding power

// Opcontrol shaping: slew limits jerk with tall stacks (tip/descore risk),
// expo gives fine control near center for goal alignment.
constexpr int OPCONTROL_SLEW_PER_LOOP = 10; // max voltage change per 20ms loop
constexpr double OPCONTROL_EXPO_GAIN = 0.4; // 0 = linear, 1 = full cubic blend

// Proportional gain correcting heading drift during drive_distance() using
// the IMU. 0 disables correction (e.g. when no IMU is connected).
constexpr double DEFAULT_HEADING_KP = 1.0;

// Basic motion profiling: caps how much drive_distance()'s output can change
// per 10ms loop (out of the -127..127 motor range), so the drivetrain ramps
// up from a dead stop instead of slamming to full power and slipping.
constexpr double DRIVE_MAX_ACCEL_PER_LOOP = 15.0;
