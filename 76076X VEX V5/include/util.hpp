#pragma once

// Small general-purpose helpers shared across the codebase. Definitions in util.cpp.
namespace util{
    void fun();

    int randRange(int min, int max);

    int sgn(double value);

    double clamp(double value, double min, double max);

    int deadband(int joystickValue, int threshold = 5); // joystick values within +/-threshold of 0 are treated as 0

    int expo(int joystickValue, double gain = 0.4); // cubic blend for fine center control: 0 = linear
    int slew(int current, int target, int maxDelta); // rate-limit output change per loop (tip/descore guard)
    int accelLimit(int current, int target, int maxDelta); // slew only when speeding up; slowing down is instant
}