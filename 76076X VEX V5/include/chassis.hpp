#pragma once
#include "api.h" 
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pid.h"

class Chassis {
    private:
        pros::MotorGroup leftMotors;
        pros::MotorGroup rightMotors;
        pros::Imu imu;

        PID drivePID; // tune these in config.hpp whenever u guys get to that
        PID turnPID;

    public:
        Chassis(pros::MotorGroup left, pros::MotorGroup right, pros::Imu imu,
                PID drivePID, PID turnPID);

        void drive_forward(int speed, bool forward); // for forward false = backward prolly u guys can change later if u want btw
        void stop();

        void drive_distance(double inches);  // straight line using drive PID
        void turn_degrees(double degrees);   // turns to angle using IMU + turn PID
};