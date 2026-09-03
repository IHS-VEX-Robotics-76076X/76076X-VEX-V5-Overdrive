#include "main.h"
#include "util.hpp"
#include <cstdlib>

namespace util {

// Seed the C RNG with the current millisecond tick so randRange() is usable.
void fun() {
    std::srand(static_cast<unsigned int>(pros::millis()));
}

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

int deadband(int joystickValue, int threshold) {
    int absValue = joystickValue < 0 ? -joystickValue : joystickValue;
    if (absValue < threshold) return 0;
    return joystickValue;
}

int expo(int joystickValue, double gain) {
    if (joystickValue > 127) joystickValue = 127;
    if (joystickValue < -127) joystickValue = -127;
    if (gain < 0.0) gain = 0.0;
    if (gain > 1.0) gain = 1.0;
    double n = static_cast<double>(joystickValue) / 127.0;
    double out = (1.0 - gain) * n + gain * n * n * n;
    return static_cast<int>(out * 127.0);
}

int slew(int current, int target, int maxDelta) {
    if (maxDelta < 0) maxDelta = -maxDelta;
    int delta = target - current;
    if (delta > maxDelta) return current + maxDelta;
    if (delta < -maxDelta) return current - maxDelta;
    return target;
}

} // namespace util