#pragma once

#include <cmath>

// pid controller - plug in kP, kI, kD and call calculate() each loop
// higher kP = snappier but more oscillation prolly
// kI fixes leftover error (keep this pretty small usually)
// kD dampens the overshoot
//
// integralCap, settleError and settleVelocity are per-instance because a
// drive PID (error measured in encoder ticks, can be thousands) and a turn
// PID (error measured in degrees, 0-180) operate on completely different
// scales and can't share one set of tuned values.

class PID {
    public:
        double kP, kI, kD;

        PID(double p, double i, double d,
            double integralCap = 300.0,
            double settleError = 5.0,
            double settleVelocity = 1.0)
            : kP(p), kI(i), kD(d),
              integralCap(integralCap),
              settleError(settleError),
              settleVelocity(settleVelocity) {}

        // measurement is the raw process variable (encoder position, IMU
        // heading, ...) that error was computed from - i.e. error == target
        // - measurement. Needed separately from error for derivative-on-
        // measurement below.
        double calculate(double error, double measurement) {
            integral += error;

            // windup guard so integral doesnt go absolutely crazy
            if (integral >  integralCap) integral =  integralCap;
            if (integral < -integralCap) integral = -integralCap;

            // Derivative-on-measurement, not derivative-on-error: a step
            // change in the target (e.g. a fresh drive_distance()/
            // turn_degrees() call right after reset()) makes the error jump
            // instantly even though the robot's actual position/heading
            // didn't move at all. Differentiating the error would read that
            // as a huge, entirely fake rate of change ("derivative kick"),
            // producing a real output spike from a target change alone.
            // Differentiating the measurement instead only reacts to how
            // the physical process variable is actually moving.
            //
            // The first calculate() after construction/reset() has no prior
            // measurement to diff against, so it seeds instead of computing
            // a (likely huge, spurious) derivative from an assumed-zero
            // baseline - the same lazy-seeding rationale used for odometry.
            if (seeded) {
                derivative = prevMeasurement - measurement;
            } else {
                derivative = 0.0;
                seeded = true;
            }
            prevMeasurement = measurement;

            return (kP * error) + (kI * integral) + (kD * derivative);
        }

        // call this between auton movements so old state doesnt bleed over
        void reset() {
            integral = 0;
            derivative = 0;
            seeded = false;
        }

        // Settled once both the error and its rate of change (derivative) are
        // small, so a fast pass through the target isn't mistaken for having
        // arrived - checking error alone lets a moving robot "settle" right
        // as it flies past the setpoint.
        bool isSettled(double error) const {
            return std::abs(error) < settleError && std::abs(derivative) < settleVelocity;
        }

    private:
        double integral        = 0;
        double prevMeasurement = 0;
        double derivative      = 0;
        bool seeded             = false;

        double integralCap;   // cap so integral doesnt explode
        double settleError;   // how close the error must get
        double settleVelocity; // how close the error's rate of change must get
};
