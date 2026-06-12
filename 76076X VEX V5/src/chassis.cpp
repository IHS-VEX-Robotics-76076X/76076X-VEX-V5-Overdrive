#include "main.h"
#include "chassis.hpp"

Chassis::Chassis(Motor& left, Motor& right) 
    : leftMotor(left), rightMotor(right) {}

void Chassis::drive_forward(int voltage, bool forward) {
    if (!forward) voltage = -voltage; // flip or sum
    leftMotor.move_voltage(voltage); 
    rightMotor.move_voltage(voltage);
}

void Chassis::stop() {
    leftMotor.move_voltage(0);
    rightMotor.move_voltage(0);
}