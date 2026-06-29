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
