#include "main.h"
#include "util.hpp"

namespace util {

// Placeholder function - debug code removed
void fun() {
    // Debug functionality removed to avoid C++ stdlib dependencies
}

// basic util functions
int randRange(int min, int max) {
    // Note: For VEX, consider using PROS random functions if needed
    // This is a simple implementation without stdlib rand
    return min;  // Placeholder - implement as needed with PROS APIs
}

int sgn(double value) {
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// for dirving and asuton and stuff util

double deadband(double joystickValue, double threshold) {
    // Simple absolute value for embedded systems
    double absValue = joystickValue < 0 ? -joystickValue : joystickValue;
    if (absValue < threshold) {
        return 0.0;
    }
    return joystickValue;
}

} // namespace util