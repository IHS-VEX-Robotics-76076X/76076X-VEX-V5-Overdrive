#include "main.h"
#include "util.hpp"
#include <cstdlib>

namespace util {

// Placeholder function - debug code removed
void fun() {
    // Seed the C RNG with the current millisecond tick so randRange() is usable.
    std::srand(static_cast<unsigned int>(pros::millis()));
}

// basic util functions
int randRange(int min, int max) {
    if (max <= min) return min;
    int range = max - min + 1;
    return min + (std::rand() % range);
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

int deadband(int joystickValue, int threshold) {
    int absValue = joystickValue < 0 ? -joystickValue : joystickValue;
    if (absValue < threshold) return 0;
    return joystickValue;
}

} // namespace util