#pragma once
#include "api.h"
#include "pid.h"
#include <vector>
#include <utility>
#include <optional>
#include <atomic>

class Chassis {
    public:
        enum class DriveSide { LEFT, RIGHT };

    private:
        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
        pros::Imu *imu; // optional IMU pointer (may be nullptr)

        PID drivePID; // tune these in config.hpp whenever u guys get to that
        PID turnPID;
        double headingKP; // corrects drift during drive_distance using the IMU (0 = no correction)

        // Dead-reckoned odometry (inches, degrees), updated by a background
        // task started with start_odometry(). Requires an IMU for heading -
        // without one, start_odometry() is a no-op.
        double odomX = 0.0;
        double odomY = 0.0;
        double odomHeading = 0.0;
        mutable pros::Mutex odomMutex;
        std::atomic<bool> odomRunning{false};
        std::optional<pros::Task> odomTask;

        void odomLoop();

    public:
        // Construct directly from two Motor objects
        Chassis(pros::Motor &left, pros::Motor &right);

        // Convenience constructors: construct from port lists.
        // Takes a vector (not initializer_list) so callers can hand it
        // config.hpp's std::array ports directly, whatever size they are.
        Chassis(const std::vector<std::int8_t>& leftPorts,
            const std::vector<std::int8_t>& rightPorts,
            pros::Imu *imu,
            PID drivePID,
            PID turnPID,
            double headingKP = 0.0);

        // Constructor without IMU (uses imu port 0 placeholder)
        Chassis(const std::vector<std::int8_t>& leftPorts,
            const std::vector<std::int8_t>& rightPorts,
            PID drivePID,
            PID turnPID);

        void drive_forward(int speed, bool forward); // for forward false = backward prolly u guys can change later if u want btw
        void drive(int leftSpeed, int rightSpeed);
        void stop();

        void drive_distance(double inches);  // straight line using drive PID (+ IMU heading correction and basic accel limiting)
        void turn_degrees(double degrees);   // turns to angle using IMU + turn PID
        void swing_turn(double degrees, DriveSide pivotSide); // turns using only one side, pivoting on the other (locked) side

        bool has_fault() const; // true if any drivetrain motor is reporting an over-temp/over-current/driver fault

        // Dead-reckoned odometry. Call start_odometry() once (e.g. from
        // initialize()) to begin tracking position; requires an IMU.
        void start_odometry();
        void stop_odometry();
        void reset_position(double x = 0.0, double y = 0.0, double headingDeg = 0.0);
        double get_x() const;
        double get_y() const;
        double get_heading() const; // degrees, IMU rotation convention (clockwise-positive)

        // Turns to face (targetX, targetY) then drives straight to it, using
        // odometry for feedback. This is sequential point-to-point ("go to
        // point") following, not curvature-based pure pursuit - verify the
        // heading-convention math on a real robot before trusting it in a match.
        void drive_to_point(double targetX, double targetY);
        void follow_path(const std::vector<std::pair<double, double>>& waypoints);
};
