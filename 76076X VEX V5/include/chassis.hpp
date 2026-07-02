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

        PID drivePID; // gains tuned via config.hpp, see DEFAULT_DRIVE_KP/KI/KD
        PID turnPID;
        double headingKP; // corrects drift during drive_distance using the IMU (0 = no correction)

        // Dead-reckoned odometry (inches, degrees), updated by a background
        // task started with start_odometry(). Requires an IMU for heading -
        // without one, start_odometry() is a no-op. (x, y) use a compass-style
        // convention matching imu->get_rotation() directly: heading 0 = +Y
        // axis, clockwise-positive (same direction turn_degrees(+degrees)
        // actually turns the robot).
        double odomX = 0.0;
        double odomY = 0.0;
        double odomHeading = 0.0;
        // odomHeading is always imu->get_rotation() + headingOffset, never the
        // raw IMU value directly - otherwise reset_position()'s headingDeg
        // would only "stick" until the next odomLoop() tick (~10ms later),
        // which then overwrites it with the IMU's own unmodified reading.
        std::atomic<double> headingOffset{0.0};
        mutable pros::Mutex odomMutex;
        std::atomic<bool> odomRunning{false};
        std::optional<pros::Task> odomTask;

        void odomLoop();

    public:
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

        // Stops the odometry task (if running) before the rest of the object
        // is torn down. Without this, destroying a Chassis whose odometry
        // task is still running would leave the background task's Task
        // destructor to just detach it (on host builds) - a thread that
        // outlives leftMotors/rightMotors/odomMutex and touches freed memory.
        // Real device builds never actually destroy the global Chassis
        // during a match, but host tests routinely construct short-lived
        // ones, so this can't be left to caller discipline.
        ~Chassis();

        void drive_forward(int speed, bool forward); // forward=false drives backward at the same speed
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
        // Declares the robot's current (x, y, headingDeg). headingDeg is kept
        // as an offset from the IMU's own rotation, so it persists (get_heading()
        // will keep reporting it, adjusted as the robot turns) rather than being
        // overwritten by the IMU's raw reading on the next odometry tick.
        void reset_position(double x = 0.0, double y = 0.0, double headingDeg = 0.0);
        double get_x() const;
        double get_y() const;
        double get_heading() const; // degrees, IMU rotation convention (clockwise-positive)

        // Turns to face (targetX, targetY) then drives straight to it, using
        // odometry for feedback. Requires start_odometry() to already be
        // running (a no-op otherwise - without it there's no live position to
        // navigate from). This is sequential point-to-point ("go to point")
        // following, not curvature-based pure pursuit - the (x, y)
        // convention is internally consistent with odometry, but which
        // physical direction on the field is "+X"/"+Y" still depends on
        // however the IMU happens to be oriented, so verify on a real robot.
        void drive_to_point(double targetX, double targetY);
        void follow_path(const std::vector<std::pair<double, double>>& waypoints);
};
