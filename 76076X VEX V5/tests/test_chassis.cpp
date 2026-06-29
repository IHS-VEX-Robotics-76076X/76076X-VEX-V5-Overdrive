#include <iostream>
#include <cassert>

#include "chassis.hpp"
#include "util.hpp"
#include "pid.h"

int main() {
    std::cout << "Host test: Chassis drive_distance and turn_degrees\n";

    // Create motors on ports 1 and 2 via host helpers
    pros::host_set_motor_position(1, 0.0);
    pros::host_set_motor_position(2, 0.0);

    pros::Motor left(1);
    pros::Motor right(2);
    pros::Imu imu(0);

    PID drivePid(0.5, 0.0, 0.0);
    PID turnPid(1.0, 0.0, 0.0);

    // MotorGroup constructor using initializer lists
    Chassis robot({1}, {2}, &imu, drivePid, turnPid);

    // Test drive distance (this will simulate movement via MotorGroup mock)
    robot.drive_distance(10.0); // 10 inches

    double leftPos = pros::host_get_motor_position(1);
    double rightPos = pros::host_get_motor_position(2);

    std::cout << "Left pos: " << leftPos << " Right pos: " << rightPos << "\n";
    assert(leftPos != 0.0 || rightPos != 0.0);

    // Test turn - set an initial imu rotation and call turn
    imu.set_rotation(0.0);
    robot.turn_degrees(90.0);
    std::cout << "IMU rotation after turn (simulated): " << imu.get_rotation() << "\n";

    std::cout << "All chassis tests ran (asserts passed if no crash).\n";
    return 0;
}
