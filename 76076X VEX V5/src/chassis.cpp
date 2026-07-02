#include "main.h"
#include "chassis.hpp"
#include "util.hpp"

#include <cmath>
#include <cstdint>
#include <cerrno>
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

Chassis::~Chassis() {
    stop_odometry();
}

void Chassis::drive_forward(int speed, bool forward) {
    if (!forward) speed = -speed;
    speed = static_cast<int>(util::clamp(speed, -127.0, 127.0));
    leftMotors.move(speed);
    rightMotors.move(speed);
}

void Chassis::drive(int leftSpeed, int rightSpeed) {
    // Callers commonly sum two independent joystick axes (e.g. arcade drive:
    // forward +/- turn), which can easily land outside the +/-127 motor
    // range even though each axis alone is in range - clamp here so every
    // caller gets a safe value instead of relying on each call site (or
    // undocumented device-side clamping) to do it.
    leftMotors.move(static_cast<int>(util::clamp(leftSpeed, -127.0, 127.0)));
    rightMotors.move(static_cast<int>(util::clamp(rightSpeed, -127.0, 127.0)));
}

void Chassis::stop() {
    leftMotors.move(0);
    rightMotors.move(0);
}

bool Chassis::has_fault() const {
    for (auto flags : leftMotors.get_faults_all()) if (flags != 0) return true;
    for (auto flags : rightMotors.get_faults_all()) if (flags != 0) return true;

    // get_faults_all()'s bitfield (over-temp/over-current/driver-fault) does
    // NOT include "disconnected" - a fully unplugged motor doesn't report a
    // fault flag at all, it just fails the next API call and sets errno.
    // get_position_all() documents ENODEV for exactly this case, so use it
    // as a cheap connectivity probe (MotorGroup has no is_installed() of its
    // own - only individual pros::Motor objects do).
    errno = 0;
    leftMotors.get_position_all();
    if (errno == ENODEV) return true;

    errno = 0;
    rightMotors.get_position_all();
    if (errno == ENODEV) return true;

    return false;
}

void Chassis::drive_distance(double inches) {
    double target = inches * TICKS_PER_INCH;

    // Measure distance relative to a captured starting position instead of
    // tare_position_all(): taring resets the physical/shared encoder
    // counters to 0, which corrupts odomLoop() if it's running concurrently
    // (it reads these same counters and expects them to be continuous, not
    // reset out from under it mid-match - every drive_distance() call after
    // the first would otherwise permanently wipe out accumulated odometry).
    std::vector<double> startLeftPositions = leftMotors.get_position_all();
    std::vector<double> startRightPositions = rightMotors.get_position_all();
    double startLeftAvg = std::accumulate(startLeftPositions.begin(), startLeftPositions.end(), 0.0) / startLeftPositions.size();
    double startRightAvg = std::accumulate(startRightPositions.begin(), startRightPositions.end(), 0.0) / startRightPositions.size();
    double startAvg = (startLeftAvg + startRightAvg) / 2.0;

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
        double current = (leftAvg + rightAvg) / 2.0 - startAvg;

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

        if (turnPID.isSettled(error)) break;
        if (pros::millis() - startTime >= TURN_TIMEOUT_MS) break; // stalled/never converging - don't hang forever

        pros::delay(10);
    }

    stop();
}

// --- Odometry -------------------------------------------------------------

void Chassis::set_tracking_wheels(pros::Rotation *leftWheel, pros::Rotation *rightWheel,
                                   pros::Rotation *backWheel, double wheelDiameterInch) {
    leftTrackingWheel = leftWheel;
    rightTrackingWheel = rightWheel;
    backTrackingWheel = backWheel;
    trackingWheelInchesPerCentidegree = (wheelDiameterInch * M_PI) / 36000.0;
}

void Chassis::odomLoop() {
    // Both parallel wheels must be attached to use tracking wheels at all;
    // the back wheel is independently optional (checked separately below).
    bool useTrackingWheels = (leftTrackingWheel != nullptr && rightTrackingWheel != nullptr);

    double prevLeft = 0.0, prevRight = 0.0, prevBack = 0.0;
    if (useTrackingWheels) {
        prevLeft = leftTrackingWheel->get_position();
        prevRight = rightTrackingWheel->get_position();
        if (backTrackingWheel != nullptr) prevBack = backTrackingWheel->get_position();
    } else {
        std::vector<double> initialLeft = leftMotors.get_position_all();
        std::vector<double> initialRight = rightMotors.get_position_all();
        prevLeft = std::accumulate(initialLeft.begin(), initialLeft.end(), 0.0) / initialLeft.size();
        prevRight = std::accumulate(initialRight.begin(), initialRight.end(), 0.0) / initialRight.size();
    }

    while (odomRunning) {
        // deltaForward: distance traveled straight ahead this tick.
        // deltaStrafe: sideways (local +X = robot's right) distance this
        // tick - only ever nonzero with a back tracking wheel attached;
        // there's no way to measure lateral drift from drive-motor encoders
        // or a single pair of forward-facing wheels at all.
        double deltaForward = 0.0;
        double deltaStrafe = 0.0;

        if (useTrackingWheels) {
            double left = leftTrackingWheel->get_position();
            double right = rightTrackingWheel->get_position();
            double deltaLeftIn = (left - prevLeft) * trackingWheelInchesPerCentidegree;
            double deltaRightIn = (right - prevRight) * trackingWheelInchesPerCentidegree;
            prevLeft = left;
            prevRight = right;
            deltaForward = (deltaLeftIn + deltaRightIn) / 2.0;

            if (backTrackingWheel != nullptr) {
                double back = backTrackingWheel->get_position();
                deltaStrafe = (back - prevBack) * trackingWheelInchesPerCentidegree;
                prevBack = back;
            }
        } else {
            // Fallback: drive motor encoders (slip-prone, but always available).
            std::vector<double> leftPositions = leftMotors.get_position_all();
            std::vector<double> rightPositions = rightMotors.get_position_all();
            double leftAvg = std::accumulate(leftPositions.begin(), leftPositions.end(), 0.0) / leftPositions.size();
            double rightAvg = std::accumulate(rightPositions.begin(), rightPositions.end(), 0.0) / rightPositions.size();

            double deltaLeftIn = (leftAvg - prevLeft) / TICKS_PER_INCH;
            double deltaRightIn = (rightAvg - prevRight) / TICKS_PER_INCH;
            prevLeft = leftAvg;
            prevRight = rightAvg;

            deltaForward = (deltaLeftIn + deltaRightIn) / 2.0;
        }

        double headingDeg = imu->get_rotation() + headingOffset.load();
        double headingRad = headingDeg * M_PI / 180.0;

        // heading is imu->get_rotation() directly - whatever direction the
        // IMU (and therefore turn_degrees) calls "positive" - so position
        // must accumulate using the matching compass-style convention
        // (0 = +Y axis, clockwise-positive), not standard math cos/sin
        // (0 = +X axis, counter-clockwise-positive). Using math convention
        // here would make odometry believe "forward" rotates the opposite
        // way turn_degrees actually turns the robot. deltaStrafe (local +X,
        // the robot's right) follows the same rotation: at heading 0,
        // sliding right moves along global +X - the mirror image of
        // deltaForward's heading-0 case of moving along global +Y.
        odomMutex.take();
        odomX += deltaForward * std::sin(headingRad) + deltaStrafe * std::cos(headingRad);
        odomY += deltaForward * std::cos(headingRad) - deltaStrafe * std::sin(headingRad);
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
    // headingOffset makes odomHeading track (imu rotation + offset) rather
    // than the raw IMU value, so the declared headingDeg actually persists
    // instead of being overwritten by the next odomLoop() tick.
    if (imu != nullptr) {
        headingOffset = headingDeg - imu->get_rotation();
    }

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
    // Without start_odometry() having been called, odomX/odomY/odomHeading
    // are just their default-initialized (or last reset_position()) values -
    // never updated. Driving off that stale state would silently move the
    // robot based on a position it isn't actually at, with no error or
    // indication anything is wrong.
    if (imu == nullptr || !odomRunning) return;

    // Read (x, y, heading) as one consistent snapshot rather than three
    // separate locked calls, which could otherwise straddle an odomLoop()
    // update between reads.
    odomMutex.take();
    double currentX = odomX;
    double currentY = odomY;
    double currentHeading = odomHeading;
    odomMutex.give();

    double dx = targetX - currentX;
    double dy = targetY - currentY;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Bearing to the target using the same compass-style convention as
    // odomLoop() (0 = +Y axis, clockwise-positive, matching imu->get_rotation()
    // directly) - this is the inverse of odomLoop's sin/cos update, so it
    // must use atan2(dx, dy), not the standard-math atan2(dy, dx).
    double angleToTarget = std::atan2(dx, dy) * 180.0 / M_PI;

    double turnAmount = angleToTarget - currentHeading;
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
