#pragma once

#include <cmath>

// pid controller - plug in kP, kI, kD and call calculate() each loop
// higher kP = snappier but more oscillation prolly
// kI fixes leftover error (keep this pretty small usually)
// kD dampens the overshoot

class PID {
    public:
        double kP, kI, kD;

        PID(double p, double i, double d)
            : kP(p), kI(i), kD(d) {}

        double calculate(double error) {
            integral += error;

            // windup guard so integral doesnt go absolutely crazy
            if (integral >  integralCap) integral =  integralCap;
            if (integral < -integralCap) integral = -integralCap;

            double derivative = error - prevError;
            prevError = error;

            return (kP * error) + (kI * integral) + (kD * derivative);
        }

        // call this between auton movements so old state doesnt bleed over
        void reset() {
            integral = 0;
            prevError = 0;
        }

        bool isSettled(double error, double threshold = 5.0) { // change threshold to whatever feels right
            return std::abs(error) < threshold;
        }

    private:
        double integral   = 0;
        double prevError  = 0;
        double integralCap = 300; // cap so it doesnt explode, tune this too
};