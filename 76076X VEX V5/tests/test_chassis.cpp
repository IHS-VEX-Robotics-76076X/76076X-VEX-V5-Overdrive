#include <iostream>
#include <cassert>
#include <cmath>

#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"
#include "pid.h"

// Generous tolerances: these check real convergence/behavior, not exact
// numbers, since the host mock's motor model is a simplified simulation.
constexpr double DISTANCE_TOLERANCE_TICKS = 10.0;
constexpr double ANGLE_TOLERANCE_DEG = 10.0;
constexpr double TIMEOUT_SLACK_MS = 500.0;

static void test_drive_distance_converges() {
    std::cout << "[test] drive_distance converges on target\n";

    pros::Imu imu(0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({1}, {2}, &imu, drivePid, turnPid);

    robot.drive_distance(10.0); // inches

    double ticksPerInch = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCH * M_PI);
    double target = 10.0 * ticksPerInch;
    double leftPos = pros::host_get_motor_position(1);
    double rightPos = pros::host_get_motor_position(2);

    std::cout << "  target=" << target << " left=" << leftPos << " right=" << rightPos << "\n";
    assert(std::abs(target - leftPos) < DISTANCE_TOLERANCE_TICKS);
    assert(std::abs(target - rightPos) < DISTANCE_TOLERANCE_TICKS);
}

static void test_turn_degrees_converges() {
    std::cout << "[test] turn_degrees converges on target angle\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({3}, {4}, &imu, drivePid, turnPid);

    robot.turn_degrees(90.0);

    std::cout << "  final IMU rotation=" << imu.get_rotation() << "\n";
    assert(std::abs(90.0 - imu.get_rotation()) < ANGLE_TOLERANCE_DEG);
}

static void test_swing_turn_only_moves_one_side() {
    std::cout << "[test] swing_turn pivots on the locked side\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({5}, {6}, &imu, drivePid, turnPid);

    robot.swing_turn(45.0, Chassis::DriveSide::LEFT);

    double leftPos = pros::host_get_motor_position(5);
    double rightPos = pros::host_get_motor_position(6);
    std::cout << "  left(pivot)=" << leftPos << " right=" << rightPos << "\n";

    assert(std::abs(leftPos) < 1.0);   // pivot side never received nonzero output
    assert(std::abs(rightPos) > 1.0);  // the other side actually moved
}

// A drivePID/turnPID with all-zero gains always outputs 0, so the motors
// never move and the error never shrinks. This is exactly the "stalled or
// mistuned gains" scenario the safety timeouts exist for - without them, both
// calls below would loop forever.
static void test_drive_and_turn_timeout_when_gains_never_converge() {
    std::cout << "[test] drive_distance/turn_degrees bail out via timeout instead of hanging\n";

    pros::Imu imu(0);
    PID zeroPid(0.0, 0.0, 0.0);
    Chassis robot({7}, {8}, &imu, zeroPid, zeroPid);

    std::uint32_t start = pros::millis();
    robot.drive_distance(10.0);
    std::uint32_t elapsedDrive = pros::millis() - start;
    std::cout << "  drive_distance elapsed=" << elapsedDrive << "ms (timeout=" << DRIVE_TIMEOUT_MS << "ms)\n";
    assert(elapsedDrive >= static_cast<std::uint32_t>(DRIVE_TIMEOUT_MS));
    assert(elapsedDrive < static_cast<std::uint32_t>(DRIVE_TIMEOUT_MS) + TIMEOUT_SLACK_MS);

    start = pros::millis();
    robot.turn_degrees(90.0);
    std::uint32_t elapsedTurn = pros::millis() - start;
    std::cout << "  turn_degrees elapsed=" << elapsedTurn << "ms (timeout=" << TURN_TIMEOUT_MS << "ms)\n";
    assert(elapsedTurn >= static_cast<std::uint32_t>(TURN_TIMEOUT_MS));
    assert(elapsedTurn < static_cast<std::uint32_t>(TURN_TIMEOUT_MS) + TIMEOUT_SLACK_MS);
}

static void test_odometry_tracks_straight_line_drive() {
    std::cout << "[test] odometry tracks position while driving straight\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({9}, {10}, &imu, drivePid, turnPid);

    robot.reset_position(0.0, 0.0, 0.0);
    robot.start_odometry();
    robot.drive_distance(12.0); // inches, heading held at 0 throughout
    pros::delay(30); // let the odometry task catch up to the final position
    robot.stop_odometry(); // must stop the background task before the test exits

    double x = robot.get_x();
    double y = robot.get_y();
    double heading = robot.get_heading();
    std::cout << "  x=" << x << " y=" << y << " heading=" << heading << "\n";

    // Heading 0 = +Y axis (compass-style convention, see Chassis::odomLoop),
    // so driving straight at heading 0 moves along Y, not X.
    assert(std::abs(y - 12.0) < 3.0);  // drove ~12in forward
    assert(std::abs(x) < 0.5);         // stayed on the starting heading
    assert(std::abs(heading) < 1.0);
}

// Unlike the straight-line test above, this forces an actual turn before
// driving - which is what catches a mismatch between the direction
// turn_degrees() rotates the robot and the direction odomLoop() thinks
// "forward" points (a real bug that was here: odomLoop used standard-math
// cos/sin on a heading value that turn_degrees treats as clockwise-positive,
// so the two disagreed about which way the robot was facing after any turn -
// invisible to a test that never turns, since both conventions agree at
// heading 0).
static void test_drive_to_point_reaches_off_axis_target() {
    std::cout << "[test] drive_to_point reaches a target that requires turning first\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({11}, {12}, &imu, drivePid, turnPid);

    robot.reset_position(0.0, 0.0, 0.0);
    robot.start_odometry();
    robot.drive_to_point(10.0, 0.0); // due +X: requires turning ~90 deg off the starting heading
    pros::delay(30);
    robot.stop_odometry();

    double x = robot.get_x();
    double y = robot.get_y();
    double heading = robot.get_heading();
    std::cout << "  x=" << x << " y=" << y << " heading=" << heading << "\n";

    assert(std::abs(x - 10.0) < 2.0);
    assert(std::abs(y) < 2.0);
    assert(std::abs(heading - 90.0) < ANGLE_TOLERANCE_DEG);
}

int main() {
    std::cout << "Host test: Chassis\n";

    test_drive_distance_converges();
    test_turn_degrees_converges();
    test_swing_turn_only_moves_one_side();
    test_drive_and_turn_timeout_when_gains_never_converge();
    test_odometry_tracks_straight_line_drive();
    test_drive_to_point_reaches_off_axis_target();

    std::cout << "All chassis tests passed.\n";
    return 0;
}
