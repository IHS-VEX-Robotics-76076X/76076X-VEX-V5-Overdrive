#include "main.h"
#include "chassis.hpp"

#include <cmath>
#include <numeric>

// change these to match ur actual robot (will move to config.hpp later)
#define WHEEL_DIAMETER  3.25   // inches - measure yours
#define GEAR_RATIO      1.0    // external gear ratio if any, otherwise leave at 1
#define TICKS_PER_REV   300.0  // 300 = blue cart, 900 = red cart, 1800 = green cart

Chassis::Chassis(pros::MotorGroup left, pros::MotorGroup right, pros::Imu imu,
                 PID drivePID, PID turnPID)
    : leftMotors(left), rightMotors(right), imu(imu),
      drivePID(drivePID), turnPID(turnPID) {}

void Chassis::drive_forward(int speed, bool forward) {
    if (!forward) speed = -speed; // flip or sum
    leftMotors.move(speed);
    rightMotors.move(speed);
}

void Chassis::stop() {
    leftMotors.move(0);
    rightMotors.move(0);
}

void Chassis::drive_distance(double inches) {
    double ticksPerInch = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER * M_PI);
    double target = inches * ticksPerInch;

    leftMotors.tare_position();
    drivePID.reset();

    while (true) {
        // average position across all left side motors
        std::vector<double> positions = leftMotors.get_position();
        double current = std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();

        double error = target - current;
        double output = drivePID.calculate(error);

        leftMotors.move(output);
        rightMotors.move(output);

        if (drivePID.isSettled(error)) break;

        pros::delay(10);
    }

    stop();
}

void Chassis::turn_degrees(double degrees) {
    double startAngle = imu.get_rotation(); // get_rotation() is unbounded unlike get_heading()
    double targetAngle = startAngle + degrees;

    turnPID.reset();

    while (true) {
        double error = targetAngle - imu.get_rotation();
        double output = turnPID.calculate(error);

        leftMotors.move(-output);  // opposite sides spin opposite ways to turn
        rightMotors.move(output);

        if (turnPID.isSettled(error)) break;

        pros::delay(10);
    }

    stop();
}