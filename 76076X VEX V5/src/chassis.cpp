#include "main.h"
#include "chassis.hpp"
#include "util.hpp"

#include <cmath>
#include <cstdint>
#include <cerrno>
#include <numeric>
#include <cassert>

// configuration/constants
#include "config.hpp"

Chassis::Chassis(const std::vector<std::int8_t>& leftPorts,
                                 const std::vector<std::int8_t>& rightPorts,
                                 pros::Imu *imu,
                                 PID drivePID, PID turnPID, double headingKP,
                                 pros::v5::MotorGears gearset)
        : leftMotors(leftPorts, gearset), rightMotors(rightPorts, gearset), imu(imu),
            drivePID(drivePID), turnPID(turnPID), headingKP(headingKP) {
    // A side with no motors would divide by zero (0.0 / 0) every time
    // drive_distance() or the odometry loop averages that side's encoder
    // readings. That produces NaN, and NaN spreads: it poisons the PID
    // error, the motor output, and the tracked position, all without
    // crashing or printing anything. The robot would just quietly stop
    // working halfway through a match with no clue why. Catch it here at
    // startup instead.
    assert(!leftPorts.empty() && !rightPorts.empty() &&
           "Chassis: leftPorts/rightPorts must not be empty - check LEFT_DRIVE_PORTS/RIGHT_DRIVE_PORTS in config.hpp");

    // HOLD makes a stopped motor actively resist being pushed, instead of
    // coasting. swing_turn() relies on this to keep the pivot side planted.
    leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
}

Chassis::Chassis(const std::vector<std::int8_t>& leftPorts,
                                 const std::vector<std::int8_t>& rightPorts,
                                 PID drivePID, PID turnPID,
                                 pros::v5::MotorGears gearset)
        : leftMotors(leftPorts, gearset), rightMotors(rightPorts, gearset), imu(nullptr),
            drivePID(drivePID), turnPID(turnPID), headingKP(0.0) {
    assert(!leftPorts.empty() && !rightPorts.empty() &&
           "Chassis: leftPorts/rightPorts must not be empty - check LEFT_DRIVE_PORTS/RIGHT_DRIVE_PORTS in config.hpp");
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
        double output = drivePID.calculate(error, current);

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
    // Every other exit path from this function (both loop-break conditions
    // below) ends by calling stop(), so a caller can rely on "the robot is
    // stopped" once turn_degrees() returns. Returning early here without
    // stopping would break that: without an IMU there's nothing this
    // function can safely command, but the drivetrain could still be
    // spinning from whatever ran before this call (e.g. straight into this
    // being the very first movement of an autonomous routine).
    if (imu == nullptr) { stop(); return; }
    double startAngle = imu->get_rotation(); // get_rotation() is unbounded unlike get_heading()
    double targetAngle = startAngle + degrees;

    turnPID.reset();
    std::uint32_t startTime = pros::millis();

    while (true) {
        double heading = imu->get_rotation();
        double error = targetAngle - heading;
        double output = turnPID.calculate(error, heading);

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
    // Same reasoning as turn_degrees() above: every other exit path ends
    // with stop(), so this one shouldn't be the exception.
    if (imu == nullptr) { stop(); return; }
    double startAngle = imu->get_rotation();
    double targetAngle = startAngle + degrees;

    turnPID.reset();
    std::uint32_t startTime = pros::millis();

    while (true) {
        double heading = imu->get_rotation();
        double error = targetAngle - heading;
        double output = turnPID.calculate(error, heading);
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

// ===========================================================================
//                          POSITION TRACKING
// ===========================================================================

void Chassis::set_tracking_wheel(pros::Rotation *wheel, double wheelDiameterInch) {
    trackingWheel = wheel;

    // A pros::Rotation sensor counts in centidegrees: 36000 of them per full
    // turn of the wheel. One full turn moves the robot one circumference,
    // which is (diameter * pi) inches. So each centidegree is worth:
    //
    //     (diameter * pi) / 36000  inches
    //
    // Working this out once here keeps it out of the loop below.
    trackingWheelInchesPerCentidegree = (wheelDiameterInch * M_PI) / 36000.0;
}

void Chassis::odomLoop() {
    // Prefer the tracking wheel when one is installed. It is unpowered, so
    // it cannot spin uselessly the way a driven wheel does when it slips.
    const bool useTrackingWheel = (trackingWheel != nullptr);

    // To work out how far we moved, we compare this tick's sensor reading
    // against last tick's. That means we need a starting reading to compare
    // against - a "baseline".
    //
    // The baseline is set from the first GOOD reading, not simply the first
    // reading. If a sensor happens to be unplugged at the exact moment
    // odometry starts (a loose wire at boot, say), its reading is a garbage
    // error value. Storing that as the baseline would corrupt every single
    // distance we calculate afterward, because every later reading gets
    // compared against garbage. Waiting for a good reading avoids that.
    bool seeded = false;
    double prevReading = 0.0;

    while (odomRunning) {
        // How far the robot moved forward since the last tick, in inches.
        // Stays 0 if this tick's sensor reading was unusable.
        double deltaForward = 0.0;

        if (useTrackingWheel) {
            // An unplugged or broken Rotation sensor does not return a small
            // or zero value - it returns PROS_ERR, a huge error code. Adding
            // that to our position would fling the robot thousands of inches
            // across the field in one tick, and there is no way to undo it
            // afterward. So we throw the reading away instead. We lose
            // whatever movement happened during the disconnection, which is
            // far better than corrupting the position permanently.
            std::int32_t raw = trackingWheel->get_position();

            if (raw != PROS_ERR) {
                double reading = raw;
                if (seeded) {
                    deltaForward = (reading - prevReading) * trackingWheelInchesPerCentidegree;
                } else {
                    seeded = true; // first good reading becomes the baseline
                }
                prevReading = reading;
            }
        } else {
            // No tracking wheel, so fall back to the drive motors' own
            // encoders. This works, but powered wheels slip, so the position
            // drifts more over the course of a match.
            //
            // We average both sides together. During a turn one side goes
            // forward while the other goes backward, so those cancel out and
            // the average correctly reports "no forward movement".
            std::vector<double> leftPositions = leftMotors.get_position_all();
            std::vector<double> rightPositions = rightMotors.get_position_all();
            double leftAvg = std::accumulate(leftPositions.begin(), leftPositions.end(), 0.0) / leftPositions.size();
            double rightAvg = std::accumulate(rightPositions.begin(), rightPositions.end(), 0.0) / rightPositions.size();
            double reading = (leftAvg + rightAvg) / 2.0;

            // Same danger as above, different error value. An unplugged motor
            // reports PROS_ERR_F, which is literally infinity. Averaging
            // anything with infinity gives infinity, so one bad motor poisons
            // the whole reading. isfinite() catches it.
            if (std::isfinite(reading)) {
                if (seeded) {
                    deltaForward = (reading - prevReading) / TICKS_PER_INCH;
                } else {
                    seeded = true;
                }
                prevReading = reading;
            }
        }

        // Which way we are pointed, straight from the IMU (plus whatever
        // offset reset_position() established).
        double headingDeg = imu->get_rotation() + headingOffset.load();
        double headingRad = headingDeg * M_PI / 180.0;

        // Break this tick's forward movement into how much of it was along
        // X and how much was along Y, based on which way we were facing.
        //
        // Note this uses sin for X and cos for Y, which looks backwards if
        // you are used to math class. That is because we use the compass
        // convention (heading 0 = +Y, clockwise positive) rather than the
        // math convention (heading 0 = +X, counter-clockwise positive). See
        // the big comment at the top of chassis.hpp. Getting this backwards
        // would make the robot's tracked position rotate the opposite way
        // from how it actually turns.
        odomMutex.take();
        odomX += deltaForward * std::sin(headingRad);
        odomY += deltaForward * std::cos(headingRad);
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

    // A target the robot is already (essentially) at has no well-defined
    // bearing - atan2(~0, ~0) still returns some angle (0 in the exact-zero
    // case), which would otherwise be read as "turn to face that direction"
    // and spin the robot toward heading 0 for no reason, burning the full
    // turn timeout in the process. Skip both the turn and the drive instead.
    if (distance < 0.5) return; // inches

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
