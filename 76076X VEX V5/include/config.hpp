#pragma once

#include <array>
#include <cstdint>
#include <cmath>

// Driver control style. ARCADE: one stick (left Y = forward, left X = turn).
// TANK: two sticks (left Y = left side, right Y = right side). This is a tank
// drivetrain (3 motors/side, no mecanum) - "arcade" here still means single-
// stick turning, not strafing.
enum class DriveMode { ARCADE, TANK };
constexpr DriveMode DEFAULT_DRIVE_MODE = DriveMode::ARCADE;

// Robot configuration constants (tune for your robot)
constexpr double WHEEL_DIAMETER_INCH = 3.25; // inches
constexpr double GEAR_RATIO = 1.0;
constexpr double TICKS_PER_REV = 300.0;

// Computed once at compile time instead of every drive_distance()/odomLoop()
// call - it's a pure function of the three constants above, so recomputing
// it at runtime on every call was redundant (and risked drifting out of
// sync if the formula ever changed in only one of the two call sites).
constexpr double TICKS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * M_PI);

// Drive motor ports: 3 motors per side.
// Left side uses reversed ports for a mirrored drivetrain.
constexpr std::array<std::int8_t, 3> LEFT_DRIVE_PORTS = {-1, -2, -3};
constexpr std::array<std::int8_t, 3> RIGHT_DRIVE_PORTS = {4, 5, 6};

// Mechanism ports.
constexpr int CASCADE_MOTOR_PORT = 7;
constexpr int INTAKE_MOTOR_PORT = 8;
constexpr int ARM_TURN_MOTOR_PORT = 9;
constexpr int CLAMP_MOTOR_PORT = 10;
constexpr int INERTIAL_SENSOR_PORT = 11;

// Tracking wheel odometry (3-wheel: two parallel + one perpendicular).
// Dedicated, non-powered wheels give slip-free position tracking - unlike
// reading the drive motors' own encoders, they aren't thrown off by wheel
// spin during acceleration or collisions. Ports use the same
// negative-for-reversed convention as motor ports (see pros::Rotation).
// Heading still comes from the IMU either way (see Chassis::odomLoop) - the
// perpendicular wheel is what actually adds new information, since it
// tracks lateral drift/scrub that encoder-only or single-wheel odometry
// can't see at all.
// TODO: set these to your actual tracking wheel ports/geometry. Pass
// nullptr for backTrackingWheel in Chassis::set_tracking_wheels() if you
// don't have a perpendicular wheel - you still get slip-free forward
// tracking from the two parallel wheels alone, just no strafe component.
constexpr std::int8_t LEFT_TRACKING_WHEEL_PORT = 12;
constexpr std::int8_t RIGHT_TRACKING_WHEEL_PORT = -13;
constexpr std::int8_t BACK_TRACKING_WHEEL_PORT = 14;
constexpr double TRACKING_WHEEL_DIAMETER_INCH = 2.75; // common VEX tracking wheel size

// pros::Rotation reports position in centidegrees (36000 per revolution) -
// this is a fixed hardware constant, not something to tune. Computed once
// at compile time for the same reason as TICKS_PER_INCH above.
constexpr double TRACKING_WHEEL_INCHES_PER_CENTIDEGREE = (TRACKING_WHEEL_DIAMETER_INCH * M_PI) / 36000.0;

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

// Proportional gain correcting heading drift during drive_distance() using
// the IMU. 0 disables correction (e.g. when no IMU is connected).
constexpr double DEFAULT_HEADING_KP = 1.0;

// Basic motion profiling: caps how much drive_distance()'s output can change
// per 10ms loop (out of the -127..127 motor range), so the drivetrain ramps
// up from a dead stop instead of slamming to full power and slipping.
constexpr double DRIVE_MAX_ACCEL_PER_LOOP = 15.0;
