#pragma once
#include "api.h"
#include "pid.h"

class Chassis {
    private:
        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
        pros::Imu *imu; // optional IMU pointer (may be nullptr)

        PID drivePID; // tune these in config.hpp whenever u guys get to that
        PID turnPID;

    public:
        Chassis(pros::MotorGroup left, pros::MotorGroup right, pros::Imu *imu,
            PID drivePID, PID turnPID);

        // Construct directly from two Motor objects
        Chassis(pros::Motor &left, pros::Motor &right);

        // Convenience constructors: construct from port lists
        Chassis(const std::initializer_list<std::int8_t>& leftPorts,
            const std::initializer_list<std::int8_t>& rightPorts,
            pros::Imu *imu,
            PID drivePID,
            PID turnPID);

        // Constructor without IMU (uses imu port 0 placeholder)
        Chassis(const std::initializer_list<std::int8_t>& leftPorts,
            const std::initializer_list<std::int8_t>& rightPorts,
            PID drivePID,
            PID turnPID);

        void drive_forward(int speed, bool forward); // for forward false = backward prolly u guys can change later if u want btw
        void drive(int leftSpeed, int rightSpeed);
        void stop();

        void drive_distance(double inches);  // straight line using drive PID
        void turn_degrees(double degrees);   // turns to angle using IMU + turn PID
};