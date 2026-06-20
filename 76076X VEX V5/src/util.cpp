#include "main.h"
#include "util.hpp"

#include <iostream>
#include <cstdlib>
#include <cmath>

// js playing arnd w/ cpp u can remove this vvv
void fun() {
    int random1 = (std::rand() % 100) + 1;
    int random2 = (std::rand() % random1) + 1;
    int random3 = (std::rand() % random2) + 1;
    
    std::cout << "roll is " << random3 << "\n";
    if (random3 == 100) std::cout << "wow 1 in a million you rolled 100" << std::endl;
}

// basic util functions
int randRange(int min, int max) {
    return min + std::rand() % ((max + 1) - min);
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
    if (std::abs(joystickValue) < threshold) {
        return 0.0;
    }
    return joystickValue;
}

