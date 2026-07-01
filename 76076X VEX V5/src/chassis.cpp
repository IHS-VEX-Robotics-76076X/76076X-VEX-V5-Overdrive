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
            drivePID(drivePID), turnPID(turnPID), headingKP(headingKP) {
    leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
}

Chassis::Chassis(const std::vector<std::int8_t>& leftPorts,
                                 const std::vector<std::int8_t>& rightPorts,
                                 PID drivePID, PID turnPID)
        : leftMotors(leftPorts), rightMotors(rightPorts), imu(nullptr),
            drivePID(drivePID), turnPID(turnPID), headingKP(0.0) {
    leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
}

Chassis::Chassis(pros::Motor &left, pros::Motor &right)
        : leftMotors(left), rightMotors(right), imu(nullptr),
            drivePID(0.0, 0.0, 0.0), turnPID(0.0, 0.0, 0.0), headingKP(0.0) {
    leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
}

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

bool Chassis::has_fault() const {
    for (auto flags : leftMotors.get_faults_all()) if (flags != 0) return true;
    for (auto flags : rightMotors.get_faults_all()) if (flags != 0) return true;
    return false;
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
    double previousOutput = 0.0;

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

        // basic trapezoidal-style motion profile: limit how much the output
        // can change per loop so the drivetrain ramps up instead of slipping
        double delta = util::clamp(clamped - previousOutput, -DRIVE_MAX_ACCEL_PER_LOOP, DRIVE_MAX_ACCEL_PER_LOOP);
        clamped = previousOutput + delta;
        previousOutput = clamped;

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

void Chassis::swing_turn(double degrees, DriveSide pivotSide) {
    if (imu == nullptr) return; // no IMU available
    double startAngle = imu->get_rotation();
    double targetAngle = startAngle + degrees;

    turnPID.reset();
    std::uint32_t startTime = pros::millis();

    while (true) {
        double error = targetAngle - imu->get_rotation();
        double output = turnPID.calculate(error);
        double clamped = util::clamp(output, -127.0, 127.0);

        // only the non-pivot side moves; the pivot side stays locked (brake
        // mode holds it in place) so the robot swings around that wheel
        if (pivotSide == DriveSide::LEFT) {
            leftMotors.move(0);
            rightMotors.move(static_cast<int>(clamped));
        } else {
            leftMotors.move(static_cast<int>(-clamped));
            rightMotors.move(0);
        }

#ifdef HOST_BUILD
        // Simulate IMU rotation change during a swing turn in host tests
        // (roughly half the rate of a point turn, since only one side moves)
        imu->set_rotation(imu->get_rotation() + clamped * 0.025);
#endif

        if (turnPID.isSettled(error)) break;
        if (pros::millis() - startTime >= TURN_TIMEOUT_MS) break; // stalled/never converging - don't hang forever

        pros::delay(10);
    }

    stop();
}

// --- Odometry -------------------------------------------------------------

void Chassis::odomLoop() {
    double ticksPerInch = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * M_PI);

    std::vector<double> initialLeft = leftMotors.get_position_all();
    std::vector<double> initialRight = rightMotors.get_position_all();
    double prevLeft = std::accumulate(initialLeft.begin(), initialLeft.end(), 0.0) / initialLeft.size();
    double prevRight = std::accumulate(initialRight.begin(), initialRight.end(), 0.0) / initialRight.size();

    while (odomRunning) {
        std::vector<double> leftPositions = leftMotors.get_position_all();
        std::vector<double> rightPositions = rightMotors.get_position_all();
        double leftAvg = std::accumulate(leftPositions.begin(), leftPositions.end(), 0.0) / leftPositions.size();
        double rightAvg = std::accumulate(rightPositions.begin(), rightPositions.end(), 0.0) / rightPositions.size();

        double deltaLeftIn = (leftAvg - prevLeft) / ticksPerInch;
        double deltaRightIn = (rightAvg - prevRight) / ticksPerInch;
        prevLeft = leftAvg;
        prevRight = rightAvg;

        double deltaCenter = (deltaLeftIn + deltaRightIn) / 2.0;
        double headingDeg = imu->get_rotation();
        double headingRad = headingDeg * M_PI / 180.0;

        odomMutex.take();
        odomX += deltaCenter * std::cos(headingRad);
        odomY += deltaCenter * std::sin(headingRad);
        odomHeading = headingDeg;
        odomMutex.give();

        pros::delay(10);
    }
}

void Chassis::start_odometry() {
    if (imu == nullptr || odomRunning) return; // odometry needs an IMU for heading
    odomRunning = true;
    odomTask.emplace([this]() { odomLoop(); });
}

void Chassis::stop_odometry() {
    odomRunning = false;
#ifdef HOST_BUILD
    // On real hardware, tasks just run for the program's lifetime and are
    // never joined. In host builds the process can actually exit, so make
    // sure odomLoop() has really stopped before this call returns - otherwise
    // it can keep running as a detached thread and touch freed memory.
    if (odomTask) odomTask->join();
#endif
}

void Chassis::reset_position(double x, double y, double headingDeg) {
    odomMutex.take();
    odomX = x;
    odomY = y;
    odomHeading = headingDeg;
    odomMutex.give();
}

double Chassis::get_x() const {
    odomMutex.take();
    double value = odomX;
    odomMutex.give();
    return value;
}

double Chassis::get_y() const {
    odomMutex.take();
    double value = odomY;
    odomMutex.give();
    return value;
}

double Chassis::get_heading() const {
    odomMutex.take();
    double value = odomHeading;
    odomMutex.give();
    return value;
}

// --- Point-to-point following ---------------------------------------------

void Chassis::drive_to_point(double targetX, double targetY) {
    if (imu == nullptr) return; // odometry requires an IMU

    double dx = targetX - get_x();
    double dy = targetY - get_y();
    double distance = std::sqrt(dx * dx + dy * dy);

    // Standard math angle (0 = +X axis, counter-clockwise positive), converted
    // to the IMU's clockwise-positive rotation convention used by get_heading().
    // Verify this sign convention against the real IMU before trusting it in a match.
    double angleToTarget = 90.0 - (std::atan2(dy, dx) * 180.0 / M_PI);

    double turnAmount = angleToTarget - get_heading();
    while (turnAmount > 180.0) turnAmount -= 360.0;
    while (turnAmount < -180.0) turnAmount += 360.0;

    turn_degrees(turnAmount);
    drive_distance(distance);
}

void Chassis::follow_path(const std::vector<std::pair<double, double>>& waypoints) {
    // Sequential go-to-point following, not curvature-based pure pursuit.
    for (const auto& point : waypoints) {
        drive_to_point(point.first, point.second);
    }
}
