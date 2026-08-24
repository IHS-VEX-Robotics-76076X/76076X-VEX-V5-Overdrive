#pragma once
#include "api.h"
#include "pid.h"
#include <vector>
#include <utility>
#include <optional>
#include <atomic>

// ===========================================================================
//                              CHASSIS
// ===========================================================================
//
// Everything to do with the drivetrain lives in this class:
//
//   - Driving and turning by hand (used by driver control)
//   - Driving and turning exact amounts using PID (used by autonomous)
//   - Tracking where the robot is on the field (odometry)
//   - Driving to a specific spot on the field
//
// This is built for a TANK drivetrain: wheels on the left side and wheels
// on the right side, all facing forward. The robot steers by spinning one
// side faster than the other. It cannot slide sideways.
//
// ---------------------------------------------------------------------------
// HOW POSITION TRACKING WORKS
// ---------------------------------------------------------------------------
//
// The robot keeps a running guess of where it is, updated 100 times a
// second in the background. It needs two pieces of information:
//
//   1. HOW FAR it has moved  -> from the tracking wheel, or if there is no
//                               tracking wheel, from the drive motors.
//   2. WHICH WAY it is facing -> always from the IMU (gyro).
//
// Put those together every few milliseconds and you can add up the robot's
// path step by step. This is called dead reckoning.
//
// Because heading always comes from the IMU, odometry does NOT work without
// one. start_odometry() simply does nothing if no IMU was given.
//
// ---------------------------------------------------------------------------
// WHICH DIRECTION IS WHICH
// ---------------------------------------------------------------------------
//
// Positions are (x, y) in inches. Heading is in degrees.
//
//   Heading 0    = facing along +Y
//   Heading 90   = facing along +X
//   Turning right (clockwise) makes the heading go UP
//
// This is compass style, like a real compass where north is 0 and east is
// 90. It is NOT the convention used in math class (where 0 points along +X
// and angles increase counter-clockwise). We use compass style because it
// matches what the IMU reports directly, so nothing has to be flipped or
// converted, and turn_degrees(+90) really does turn the robot right.
//
// One caveat: which physical corner of the field counts as "+X" depends
// entirely on how the IMU is mounted and where the robot starts. Always
// check this on the real robot before trusting it in a match.
//
// ===========================================================================

class Chassis {
    public:
        // Which side stays still during a swing turn.
        enum class DriveSide { LEFT, RIGHT };

    private:
        // ---- Hardware -----------------------------------------------------
        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
        pros::Imu *imu; // may be nullptr - the robot still drives, just without
                        // heading correction, turning, or odometry

        // ---- Control ------------------------------------------------------
        PID drivePID;     // used by drive_distance()
        PID turnPID;      // used by turn_degrees() and swing_turn()
        double headingKP; // keeps drive_distance() going straight (0 = off)

        // ---- Tracking wheel -----------------------------------------------
        // One unpowered wheel, mounted facing forward, that measures how far
        // the robot travels. nullptr means "not installed", in which case
        // odometry falls back to reading the drive motors' own encoders.
        pros::Rotation *trackingWheel = nullptr;

        // A pros::Rotation sensor reports its position in centidegrees
        // (36000 per full turn). This is the conversion factor from those
        // to inches of travel, worked out once in set_tracking_wheel().
        double trackingWheelInchesPerCentidegree = 0.0;

        // ---- Position -----------------------------------------------------
        // Where the robot thinks it is. Updated by the background odometry
        // task, so everything here is shared between two tasks at once.
        double odomX = 0.0;
        double odomY = 0.0;
        double odomHeading = 0.0;

        // Heading is stored as (what the IMU reports) + (this offset), not
        // as the raw IMU value. Without the offset, reset_position()'s
        // heading would be wiped out by the next odometry update ~10ms
        // later, which would overwrite it with the IMU's own reading.
        std::atomic<double> headingOffset{0.0};

        // Guards odomX/odomY/odomHeading. Anyone reading position gets all
        // three from the same moment in time, never a half-updated mix of
        // an old x with a new y.
        mutable pros::Mutex odomMutex;

        std::atomic<bool> odomRunning{false};
        std::optional<pros::Task> odomTask;

        // The background loop itself. Runs until odomRunning goes false.
        void odomLoop();

    public:
        // -------------------------------------------------------------------
        // SETUP
        // -------------------------------------------------------------------

        // Builds a drivetrain from a list of ports per side, with an IMU.
        //
        //   leftPorts / rightPorts - motor ports, negative to reverse
        //   imu                    - the inertial sensor, for turns/odometry
        //   drivePID / turnPID     - tuning for driving and turning
        //   headingKP              - drift correction strength (0 = off)
        //   gearset                - motor cartridge color (see config.hpp)
        Chassis(const std::vector<std::int8_t>& leftPorts,
            const std::vector<std::int8_t>& rightPorts,
            pros::Imu *imu,
            PID drivePID,
            PID turnPID,
            double headingKP = 0.0,
            pros::v5::MotorGears gearset = pros::v5::MotorGears::blue);

        // Same, but with no IMU. The robot can still drive forward and
        // backward, but turn_degrees(), swing_turn(), odometry, and
        // drive_to_point() will not work.
        Chassis(const std::vector<std::int8_t>& leftPorts,
            const std::vector<std::int8_t>& rightPorts,
            PID drivePID,
            PID turnPID,
            pros::v5::MotorGears gearset = pros::v5::MotorGears::blue);

        // Shuts down the background odometry task before anything else gets
        // torn down. On a real robot the Chassis lives for the whole match
        // and is never destroyed, but the host tests create and destroy
        // them constantly, and a leftover background task would keep
        // touching memory that no longer exists.
        ~Chassis();

        // -------------------------------------------------------------------
        // DRIVING BY HAND (driver control)
        // -------------------------------------------------------------------
        // These send power straight to the motors with no feedback. Power
        // ranges from -127 (full reverse) to 127 (full forward), and is
        // clamped automatically so an out-of-range number is never a
        // problem.

        void drive_forward(int speed, bool forward); // both sides together
        void drive(int leftSpeed, int rightSpeed);   // each side separately
        void stop();                                  // both sides to zero

        // -------------------------------------------------------------------
        // DRIVING EXACT AMOUNTS (autonomous)
        // -------------------------------------------------------------------
        // These block until the robot arrives, or until the safety timeout
        // in config.hpp runs out. They always stop the motors before
        // returning, so after any of these the robot is guaranteed still.

        // Drives straight for a distance in inches. Negative goes backward.
        // Uses the IMU (if present) to hold a straight line, and ramps power
        // up gradually so the wheels do not spin out.
        void drive_distance(double inches);

        // Turns in place by an angle in degrees. Positive turns right
        // (clockwise). Needs an IMU - without one this just stops the
        // motors and returns.
        void turn_degrees(double degrees);

        // Turns by powering only ONE side while the other stays locked, so
        // the robot pivots around the locked wheel instead of spinning in
        // place. pivotSide is the side that stays still. Needs an IMU.
        void swing_turn(double degrees, DriveSide pivotSide);

        // -------------------------------------------------------------------
        // HEALTH CHECK
        // -------------------------------------------------------------------

        // True if any drivetrain motor is overheating, drawing too much
        // current, reporting a driver fault, or has been unplugged entirely.
        bool has_fault() const;

        // -------------------------------------------------------------------
        // POSITION TRACKING
        // -------------------------------------------------------------------

        // Attaches the forward-facing tracking wheel. Call this BEFORE
        // start_odometry(). If you never call it, or pass nullptr, odometry
        // falls back to the drive motor encoders instead - which still
        // works, but drifts more because powered wheels slip.
        void set_tracking_wheel(pros::Rotation *wheel, double wheelDiameterInch);

        // Starts and stops the background position-tracking task. Call
        // start_odometry() once, normally at the end of initialize().
        // Does nothing at all if there is no IMU.
        void start_odometry();
        void stop_odometry();

        // Tells the robot where it currently is. Use this at the start of
        // autonomous to declare the starting position on the field.
        void reset_position(double x = 0.0, double y = 0.0, double headingDeg = 0.0);

        // Reads the current position. Safe to call from any task.
        double get_x() const;
        double get_y() const;
        double get_heading() const;

        // -------------------------------------------------------------------
        // DRIVING TO A SPOT ON THE FIELD
        // -------------------------------------------------------------------

        // Turns to face (x, y), then drives straight to it. Requires
        // start_odometry() to already be running, since it needs to know
        // where the robot currently is. Does nothing if the robot is
        // already there.
        //
        // This is simple "turn, then go" movement. It does not follow a
        // smooth curve, and it will not avoid obstacles.
        void drive_to_point(double targetX, double targetY);

        // Runs drive_to_point() for each spot in the list, in order.
        void follow_path(const std::vector<std::pair<double, double>>& waypoints);
};
