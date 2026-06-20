#pragma once

/** 
 * can you guys like add includes and stuff and also create teh definitions in like another file 
 */
namespace util{
    void fun();

    int randRange(int min, int max);

    int sgn(double value);

    double clamp(double value, double min, double max);

    double deadband(double joystickValue, double threshold = 5.0); // change the threshold here
}