#include <iostream>
#include <cassert>
#include <cmath>
#include <atomic>
#include <thread>

#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"
#include "pid.h"

// Generous tolerances: these check real convergence/behavior, not exact
// numbers, since the host mock's motor model is a simplified simulation.
constexpr double DISTANCE_TOLERANCE_TICKS = 10.0;
constexpr double ANGLE_TOLERANCE_DEG = 10.0;
constexpr double TIMEOUT_SLACK_MS = 500.0;

// The real IMU obviously doesn't respond to fake motors, so something has to
// simulate "the robot rotated" for turn_degrees()/swing_turn() to have
// anything to converge against in host tests. This used to live inside
// Chassis itself (a #ifdef HOST_BUILD block in turn_degrees()/swing_turn());
// it lives here instead so production chassis logic has no simulation-only
// branches at all.
//
// Derives the rotation change from the actual left/right motor voltage
// difference, which naturally reproduces both a point turn (opposite
// voltages) and a swing turn (one side at 0) at the correct relative rate
// using a single formula, rather than two separately hand-picked constants.
class ImuTurnSimulator {
    public:
        ImuTurnSimulator(pros::Imu& imu, int leftPort, int rightPort)
            : imu_(imu), leftPort_(leftPort), rightPort_(rightPort),
              running_(true), thread_([this] { run(); }) {}

        ~ImuTurnSimulator() {
            running_ = false;
            if (thread_.joinable()) thread_.join();
        }

    private:
        void run() {
            while (running_) {
                int leftVoltage = pros::host_get_motor_voltage(leftPort_);
                int rightVoltage = pros::host_get_motor_voltage(rightPort_);
                imu_.set_rotation(imu_.get_rotation() + (rightVoltage - leftVoltage) * 0.025);
                pros::delay(10);
            }
        }

        pros::Imu& imu_;
        int leftPort_;
        int rightPort_;
        std::atomic<bool> running_;
        std::thread thread_;
};

static void test_drive_distance_converges() {
    std::cout << "[test] drive_distance converges on target\n";

    pros::Imu imu(0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({1}, {2}, &imu, drivePid, turnPid);

    robot.drive_distance(10.0); // inches

    double target = 10.0 * TICKS_PER_INCH;
    double leftPos = pros::host_get_motor_position(1);
    double rightPos = pros::host_get_motor_position(2);

    std::cout << "  target=" << target << " left=" << leftPos << " right=" << rightPos << "\n";
    assert(std::abs(target - leftPos) < DISTANCE_TOLERANCE_TICKS);
    assert(std::abs(target - rightPos) < DISTANCE_TOLERANCE_TICKS);
}

// drive()/drive_forward() used to pass their inputs straight to the motors
// with no clamping. That's fine for a single joystick axis (already in
// range), but opcontrol's arcade mixing sums two independent axes
// (forward +/- turn), which routinely exceeds +/-127 whenever a driver
// pushes both sticks hard - a completely ordinary driving scenario, not an
// edge case.
static void test_drive_and_drive_forward_clamp_output() {
    std::cout << "[test] drive()/drive_forward() clamp output to +/-127\n";

    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({40}, {41}, drivePid, turnPid);

    robot.drive(200, -200); // e.g. forward=100, turn=100 in arcade mixing
    int leftVoltage = pros::host_get_motor_voltage(40);
    int rightVoltage = pros::host_get_motor_voltage(41);
    std::cout << "  drive(200, -200): left=" << leftVoltage << " right=" << rightVoltage << "\n";
    assert(leftVoltage == 127);
    assert(rightVoltage == -127);

    robot.drive_forward(500, true);
    int forwardVoltage = pros::host_get_motor_voltage(40);
    std::cout << "  drive_forward(500, true): left=" << forwardVoltage << "\n";
    assert(forwardVoltage == 127);
}

// has_fault() used to only check get_faults_all()'s bitfield (over-temp/
// over-current/driver-fault), which does NOT include "fully disconnected" -
// a real unplugged motor doesn't set any fault flag, it just fails the next
// API call and sets errno = ENODEV. This verifies has_fault() actually
// catches that case now, using the host mock's disconnection simulation.
static void test_has_fault_detects_disconnected_motor() {
    std::cout << "[test] has_fault() detects a disconnected motor via errno\n";

    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({42}, {43}, drivePid, turnPid);

    assert(!robot.has_fault()); // nothing disconnected yet

    pros::host_set_motor_connected(42, false);
    std::cout << "  after disconnecting port 42: has_fault()=" << robot.has_fault() << "\n";
    assert(robot.has_fault());

    pros::host_set_motor_connected(42, true); // reconnect - don't leak state into later tests
    std::cout << "  after reconnecting: has_fault()=" << robot.has_fault() << "\n";
    assert(!robot.has_fault());
}

static void test_turn_degrees_converges() {
    std::cout << "[test] turn_degrees converges on target angle\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({3}, {4}, &imu, drivePid, turnPid);

    ImuTurnSimulator sim(imu, 3, 4);
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

    ImuTurnSimulator sim(imu, 5, 6);
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

// Tracking wheels are unpowered and independent of the drive motors in the
// mock (as on real hardware), so these tests drive them directly via
// set_position() rather than through drive_distance() - that isolates the
// odomLoop() math itself (does a tracking-wheel delta convert to (x, y)
// correctly?) from whether the drivetrain happens to move in sync with them.
static void test_tracking_wheel_odometry_pure_forward() {
    std::cout << "[test] tracking wheel odometry: pure forward movement\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0); // heading 0 throughout
    pros::Rotation leftWheel(60);
    pros::Rotation rightWheel(61);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({62}, {63}, &imu, drivePid, turnPid); // drive ports unused - tracking wheels take over

    robot.set_tracking_wheels(&leftWheel, &rightWheel, nullptr, TRACKING_WHEEL_DIAMETER_INCH);
    robot.reset_position(0.0, 0.0, 0.0);
    robot.start_odometry();
    pros::delay(30); // let odomLoop seed its baseline

    double targetInches = 10.0;
    double centideg = targetInches / TRACKING_WHEEL_INCHES_PER_CENTIDEGREE;
    leftWheel.set_position(centideg);
    rightWheel.set_position(centideg);
    pros::delay(30); // let odomLoop pick up the change

    robot.stop_odometry();

    double x = robot.get_x();
    double y = robot.get_y();
    std::cout << "  x=" << x << " y=" << y << " (expect x~0, y~10)\n";
    assert(std::abs(y - 10.0) < 0.5);
    assert(std::abs(x) < 0.5);
}

// Only the back (perpendicular) tracking wheel moves here - this is exactly
// the kind of lateral drift/scrub that drive-motor-encoder odometry (or even
// two forward-only tracking wheels) has no way to detect at all.
static void test_tracking_wheel_odometry_pure_strafe() {
    std::cout << "[test] tracking wheel odometry: pure strafe (back wheel)\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    pros::Rotation leftWheel(64);
    pros::Rotation rightWheel(65);
    pros::Rotation backWheel(66);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({67}, {68}, &imu, drivePid, turnPid);

    robot.set_tracking_wheels(&leftWheel, &rightWheel, &backWheel, TRACKING_WHEEL_DIAMETER_INCH);
    robot.reset_position(0.0, 0.0, 0.0);
    robot.start_odometry();
    pros::delay(30);

    double targetInches = 5.0;
    double centideg = targetInches / TRACKING_WHEEL_INCHES_PER_CENTIDEGREE;
    backWheel.set_position(centideg);
    pros::delay(30);

    robot.stop_odometry();

    double x = robot.get_x();
    double y = robot.get_y();
    std::cout << "  x=" << x << " y=" << y << " (expect x~5, y~0)\n";
    assert(std::abs(x - 5.0) < 0.5);
    assert(std::abs(y) < 0.5);
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

    ImuTurnSimulator sim(imu, 11, 12);
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

// drive_to_point()/follow_path() used to have no way of knowing whether
// odometry was actually running - without start_odometry(), odomX/odomY are
// just their default (0, 0), so calling drive_to_point() would silently
// drive the robot based on stale position data with no error or indication
// anything was wrong. Confirmed empirically before fixing: the robot
// physically moved (nonzero encoder deltas) even though get_x()/get_y()
// still reported (0, 0) afterward. It should now be a safe no-op instead.
static void test_drive_to_point_without_odometry_is_a_no_op() {
    std::cout << "[test] drive_to_point without start_odometry() doesn't move the robot\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({52}, {53}, &imu, drivePid, turnPid);

    // deliberately never call start_odometry()
    robot.drive_to_point(10.0, 10.0);

    double leftPos = pros::host_get_motor_position(52);
    double rightPos = pros::host_get_motor_position(53);
    std::cout << "  left=" << leftPos << " right=" << rightPos << " (expect 0, 0 - no-op)\n";
    assert(leftPos == 0.0);
    assert(rightPos == 0.0);
}

// reset_position()'s headingDeg used to only "stick" until the next
// odomLoop() tick (~10ms later), which unconditionally overwrote odomHeading
// with the IMU's raw, unmodified rotation - silently discarding whatever
// heading the caller had just declared. Checking immediately after the call
// (like test_odometry_tracks_straight_line_drive does, implicitly, by
// starting from heading 0 == the IMU's own default) can't catch this; this
// test declares a *nonzero* heading and waits past several odometry ticks.
static void test_reset_position_heading_persists() {
    std::cout << "[test] reset_position's heading persists past the next odometry tick\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0); // IMU itself still thinks it's at 0
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({19}, {20}, &imu, drivePid, turnPid);

    robot.start_odometry();
    robot.reset_position(5.0, 5.0, 45.0); // declare "I'm at (5,5) facing 45 degrees"
    pros::delay(50); // several odometry ticks - the bug would revert this to 0 within ~10ms

    double heading = robot.get_heading();
    std::cout << "  heading=" << heading << " (expect ~45, not reverted to the IMU's raw 0)\n";
    assert(std::abs(heading - 45.0) < 1.0);

    // and it should keep tracking further rotation relative to that new
    // reference, not just freeze at the declared value
    imu.set_rotation(10.0);
    pros::delay(30);
    double headingAfterTurn = robot.get_heading();
    std::cout << "  heading after +10deg IMU turn=" << headingAfterTurn << " (expect ~55)\n";
    assert(std::abs(headingAfterTurn - 55.0) < 1.0);

    robot.stop_odometry();
}

// drive_distance() used to call tare_position_all() on every call, which
// resets the shared physical/simulated encoder counters to 0. odomLoop()
// (running continuously in the background once start_odometry() is called,
// as it would be for the whole match) reads those same counters and expects
// them to be continuous - a tare mid-match would yank them out from under
// its baseline, producing one huge spurious jump that permanently corrupts
// the accumulated position. This only shows up across multiple drive_distance
// calls while odometry is running (the single-call tests above can't catch
// it), which is exactly the realistic autonomous usage pattern.
static void test_odometry_survives_multiple_drive_calls() {
    std::cout << "[test] odometry position isn't corrupted by a second drive_distance call\n";

    pros::Imu imu(0);
    imu.set_rotation(0.0);
    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);
    Chassis robot({13}, {14}, &imu, drivePid, turnPid);

    robot.reset_position(0.0, 0.0, 0.0);
    robot.start_odometry();

    robot.drive_distance(10.0);
    pros::delay(30);
    double yAfterFirstLeg = robot.get_y();
    std::cout << "  after leg 1: y=" << yAfterFirstLeg << "\n";
    assert(std::abs(yAfterFirstLeg - 10.0) < 3.0);

    robot.drive_distance(10.0); // a second leg, same heading - must not reset odometry's baseline
    pros::delay(30);
    double yAfterSecondLeg = robot.get_y();
    std::cout << "  after leg 2: y=" << yAfterSecondLeg << "\n";

    robot.stop_odometry();

    // The second leg must add to the first, not overwrite/erase it.
    assert(yAfterSecondLeg > yAfterFirstLeg + 5.0);
    assert(std::abs(yAfterSecondLeg - 20.0) < 5.0);
}

// Deliberately does NOT call stop_odometry() before robot goes out of scope.
// Chassis's destructor must stop the background task itself - otherwise (on
// host builds) the task's underlying thread just gets detached and keeps
// running past the point where leftMotors/rightMotors/odomMutex are
// destroyed, touching freed memory. This is the same crash pattern that
// showed up the first time this feature was built and tested end to end.
static void test_chassis_destructor_stops_odometry_without_crashing() {
    std::cout << "[test] Chassis destructor stops a still-running odometry task\n";

    {
        pros::Imu imu(0);
        PID drivePid(0.5, 0.0, 0.0);
        PID turnPid(1.0, 0.0, 0.0);
        Chassis robot({17}, {18}, &imu, drivePid, turnPid);

        robot.start_odometry();
        robot.drive_distance(5.0);
        // no stop_odometry() call - ~Chassis() must handle it
    }

    std::cout << "  (no crash - destructor stopped the task)\n";
}

int main() {
    std::cout << "Host test: Chassis\n";

    test_drive_distance_converges();
    test_drive_and_drive_forward_clamp_output();
    test_has_fault_detects_disconnected_motor();
    test_turn_degrees_converges();
    test_swing_turn_only_moves_one_side();
    test_drive_and_turn_timeout_when_gains_never_converge();
    test_odometry_tracks_straight_line_drive();
    test_tracking_wheel_odometry_pure_forward();
    test_tracking_wheel_odometry_pure_strafe();
    test_drive_to_point_reaches_off_axis_target();
    test_drive_to_point_without_odometry_is_a_no_op();
    test_reset_position_heading_persists();
    test_odometry_survives_multiple_drive_calls();
    test_chassis_destructor_stops_odometry_without_crashing();

    std::cout << "All chassis tests passed.\n";
    return 0;
}
