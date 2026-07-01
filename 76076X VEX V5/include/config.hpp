#pragma once

#include <array>
#include <cstdint>

// Robot configuration constants (tune for your robot)
constexpr double WHEEL_DIAMETER_INCH = 3.25; // inches
constexpr double GEAR_RATIO = 1.0;
constexpr double TICKS_PER_REV = 300.0;

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
