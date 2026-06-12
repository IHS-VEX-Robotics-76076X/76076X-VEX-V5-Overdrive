#pragma once
#include "api.h" 
#include "pros/motors.hpp"

class Chassis {
    private:
        Motor& leftMotor;
        Motor& rightMotor;

    public:
        Chassis(Motor& left, Motor& right);

        void drive_forward(int voltage, bool forward); // for forward false = backward prolly u guys can change later if u want btw
        void stop();
};