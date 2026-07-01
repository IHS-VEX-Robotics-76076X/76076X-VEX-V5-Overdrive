#pragma once
#include "api.h"
#include "pid.h"
#include <vector>

class Chassis {
    private:
        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
        pros::Imu *imu; // optional IMU pointer (may be nullptr)

        PID drivePID; // tune these in config.hpp whenever u guys get to that
        PID turnPID;
        double headingKP; // corrects drift during drive_distance using the IMU (0 = no correction)

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

        void drive_distance(double inches);  // straight line using drive PID (+ IMU heading correction if available)
        void turn_degrees(double degrees);   // turns to angle using IMU + turn PID
};