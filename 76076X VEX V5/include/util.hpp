#pragma once

// ===========================================================================
//                          SMALL HELPERS
// ===========================================================================
//
// Little math utilities used all over the codebase. The actual code for
// these is in src/util.cpp.
//
// ===========================================================================

namespace util{
    // Seeds the random number generator. Call once at startup, before using
    // randRange().
    void fun();

    // Random whole number between min and max, including both ends.
    // Returns min if the range is backwards or empty.
    int randRange(int min, int max);

    // The sign of a number: 1 if positive, -1 if negative, 0 if zero.
    int sgn(double value);

    // Forces a value to stay between min and max. Anything lower becomes
    // min, anything higher becomes max. Used constantly to keep motor power
    // inside the legal -127 to 127 range.
    double clamp(double value, double min, double max);

    // Ignores tiny joystick readings, treating anything within +/-threshold
    // of zero as exactly zero.
    //
    // Why this is needed: a released joystick rarely reads exactly 0. It
    // drifts a point or two. Without a deadband the robot slowly creeps
    // around on its own whenever nobody is touching the controller.
    int deadband(int joystickValue, int threshold = 5);
}