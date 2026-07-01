#include "main.h"
#include "chassis.hpp"
#include "util.hpp"

#include <cmath>
#include <cstdint>
#include <numeric>

// configuration/constants
#include "config.hpp"

Chassis::Chassis(const std::vector<std::int8_t>& leftPorts,
                                 const std::vector<std::int8_t>& rightPorts,
                                 pros::Imu *imu,
                                 PID drivePID, PID turnPID, double headingKP)
        : leftMotors(leftPorts), rightMotors(rightPorts), imu(imu),
            drivePID(drivePID), turnPID(turnPID), headingKP(headingKP) {}

Chassis::Chassis(const std::vector<std::int8_t>& leftPorts,
                                 const std::vector<std::int8_t>& rightPorts,
                                 PID drivePID, PID turnPID)
        : leftMotors(leftPorts), rightMotors(rightPorts), imu(nullptr),
            drivePID(drivePID), turnPID(turnPID), headingKP(0.0) {}

Chassis::Chassis(pros::Motor &left, pros::Motor &right)
        : leftMotors(left), rightMotors(right), imu(nullptr),
            drivePID(0.0, 0.0, 0.0), turnPID(0.0, 0.0, 0.0), headingKP(0.0) {}

void Chassis::drive_forward(int speed, bool forward) {
    if (!forward) speed = -speed; // flip or sum
    leftMotors.move(speed);
    rightMotors.move(speed);
}

void Chassis::drive(int leftSpeed, int rightSpeed) {
    leftMotors.move(leftSpeed);
    rightMotors.move(rightSpeed);
}

void Chassis::stop() {
    leftMotors.move(0);
    rightMotors.move(0);
}

void Chassis::drive_distance(double inches) {
    double ticksPerInch = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * M_PI);
    double target = inches * ticksPerInch;

    leftMotors.tare_position_all();
    rightMotors.tare_position_all();
    drivePID.reset();

    // hold whatever heading we started at, if an IMU is connected
    double startHeading = (imu != nullptr) ? imu->get_rotation() : 0.0;
    std::uint32_t startTime = pros::millis();

    while (true) {
        // average position across both sides so one side stalling, slipping,
        // or being under more load than the other doesn't go unnoticed
        std::vector<double> leftPositions = leftMotors.get_position_all();
        std::vector<double> rightPositions = rightMotors.get_position_all();
        double leftAvg = std::accumulate(leftPositions.begin(), leftPositions.end(), 0.0) / leftPositions.size();
        double rightAvg = std::accumulate(rightPositions.begin(), rightPositions.end(), 0.0) / rightPositions.size();
        double current = (leftAvg + rightAvg) / 2.0;

        double error = target - current;
        double output = drivePID.calculate(error);

        // clamp to motor move range (-127..127)
        double clamped = util::clamp(output, -127.0, 127.0);

        // steer back toward the starting heading if we've drifted off it
        double correction = 0.0;
        if (imu != nullptr) {
            double headingError = imu->get_rotation() - startHeading;
            correction = headingKP * headingError;
        }

        leftMotors.move(static_cast<int>(util::clamp(clamped + correction, -127.0, 127.0)));
        rightMotors.move(static_cast<int>(util::clamp(clamped - correction, -127.0, 127.0)));

        if (drivePID.isSettled(error)) break;
        if (pros::millis() - startTime >= DRIVE_TIMEOUT_MS) break; // stalled/never converging - don't hang forever

        pros::delay(10);
    }

    stop();
}

void Chassis::turn_degrees(double degrees) {
    if (imu == nullptr) return; // no IMU available
    double startAngle = imu->get_rotation(); // get_rotation() is unbounded unlike get_heading()
    double targetAngle = startAngle + degrees;

    turnPID.reset();
    std::uint32_t startTime = pros::millis();

    while (true) {
        double error = targetAngle - imu->get_rotation();
        double output = turnPID.calculate(error);

        double clamped = util::clamp(output, -127.0, 127.0);
        leftMotors.move(static_cast<int>(-clamped));  // opposite sides spin opposite ways to turn
        rightMotors.move(static_cast<int>(clamped));

#ifdef HOST_BUILD
        // Simulate IMU rotation change during a turn in host tests
        imu->set_rotation(imu->get_rotation() + clamped * 0.05);
#endif

        if (turnPID.isSettled(error)) break;
        if (pros::millis() - startTime >= TURN_TIMEOUT_MS) break; // stalled/never converging - don't hang forever

        pros::delay(10);
    }

    stop();
}