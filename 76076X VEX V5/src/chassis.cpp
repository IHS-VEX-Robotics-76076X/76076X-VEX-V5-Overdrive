#include "main.h"
#include "chassis.hpp"
#include "util.hpp"

#include <cmath>
#include <numeric>

// configuration/constants
#include "config.hpp"

Chassis::Chassis(const std::initializer_list<std::int8_t>& leftPorts,
                                 const std::initializer_list<std::int8_t>& rightPorts,
                                 pros::Imu *imu,
                                 PID drivePID, PID turnPID)
        : leftMotors(leftPorts), rightMotors(rightPorts), imu(imu),
            drivePID(drivePID), turnPID(turnPID) {}

Chassis::Chassis(const std::initializer_list<std::int8_t>& leftPorts,
                                 const std::initializer_list<std::int8_t>& rightPorts,
                                 PID drivePID, PID turnPID)
        : leftMotors(leftPorts), rightMotors(rightPorts), imu(nullptr),
            drivePID(drivePID), turnPID(turnPID) {}

Chassis::Chassis(pros::Motor &left, pros::Motor &right)
        : leftMotors(left), rightMotors(right), imu(nullptr),
            drivePID(0.0, 0.0, 0.0), turnPID(0.0, 0.0, 0.0) {}

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
    drivePID.reset();

    while (true) {
        // average position across all left side motors
        std::vector<double> positions = leftMotors.get_position_all();
        double current = std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();

        double error = target - current;
        double output = drivePID.calculate(error);

        // clamp to motor move range (-127..127)
        double clamped = util::clamp(output, -127.0, 127.0);
        leftMotors.move(static_cast<int>(clamped));
        rightMotors.move(static_cast<int>(clamped));

        if (drivePID.isSettled(error)) break;

        pros::delay(10);
    }

    stop();
}

void Chassis::turn_degrees(double degrees) {
    if (imu == nullptr) return; // no IMU available
    double startAngle = imu->get_rotation(); // get_rotation() is unbounded unlike get_heading()
    double targetAngle = startAngle + degrees;

    turnPID.reset();

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

        pros::delay(10);
    }

    stop();
}