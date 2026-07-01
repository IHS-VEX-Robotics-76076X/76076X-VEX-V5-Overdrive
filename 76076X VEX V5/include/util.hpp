#pragma once

// Small general-purpose helpers shared across the codebase. Definitions in util.cpp.
namespace util{
    void fun();

    int randRange(int min, int max);

    int sgn(double value);

    double clamp(double value, double min, double max);

    int deadband(int joystickValue, int threshold = 5); // joystick values within +/-threshold of 0 are treated as 0
}