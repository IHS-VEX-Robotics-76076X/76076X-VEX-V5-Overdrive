#include <iostream>
#include <cassert>
#include <cmath>

#include "pid.h"

// derivative-on-error would compute derivative = error - prevError, and
// prevError starts at 0 - so the very first calculate() after a fresh
// target (reset() then a step in setpoint) reads the entire step itself as
// a derivative, even though the measurement hasn't moved yet. That's a real
// output spike from nothing but a target change ("derivative kick").
// derivative-on-measurement can't do that: it seeds from the first
// measurement instead of assuming a zero baseline, so it reports 0 rate of
// change until it's actually seen the measurement move.
static void test_no_derivative_kick_on_first_call_after_reset() {
    std::cout << "[test] no derivative kick on the first calculate() after reset()\n";

    PID pid(0.0, 0.0, 5.0); // kD-only, isolates the derivative term
    double output = pid.calculate(90.0, 0.0); // target just stepped to 90; measurement still 0
    std::cout << "  output=" << output << " (expect 0)\n";
    assert(output == 0.0);

    pid.reset();
    double output2 = pid.calculate(-40.0, 12.0); // a different step after reset()
    std::cout << "  output after reset()=" << output2 << " (expect 0)\n";
    assert(output2 == 0.0);
}

// Once seeded, the derivative term should react to the measurement actually
// moving, damping the output as it approaches the target - not just always
// reading 0.
static void test_derivative_dampens_as_measurement_moves() {
    std::cout << "[test] derivative term reacts to measurement movement after seeding\n";

    PID pid(0.0, 0.0, 5.0);
    pid.calculate(90.0, 0.0);              // seed at measurement=0
    double output = pid.calculate(50.0, 40.0); // measurement rose by 40 toward the target

    // derivative = prevMeasurement - measurement = 0 - 40 = -40; kD * -40 = -200
    std::cout << "  output=" << output << " (expect -200 - damping as the measurement rises)\n";
    assert(std::abs(output - (-200.0)) < 1e-9);
}

// Sanity check that kP/kI/settling logic (unrelated to the derivative
// change) still works as a basic proportional controller.
static void test_proportional_only_controller_converges_toward_zero_error() {
    std::cout << "[test] a kP-only PID pushes error toward zero over repeated calls\n";

    PID pid(0.5, 0.0, 0.0, 300.0, 0.5, 100.0);
    double measurement = 0.0;
    double target = 100.0;
    for (int i = 0; i < 200; i++) {
        double error = target - measurement;
        double output = pid.calculate(error, measurement);
        measurement += output * 0.1; // toy plant: measurement drifts toward the commanded output
    }
    double finalError = target - measurement;
    std::cout << "  final error=" << finalError << " (expect small, converged toward 0)\n";
    assert(std::abs(finalError) < 5.0);
}

int main() {
    std::cout << "Host test: PID\n";

    test_no_derivative_kick_on_first_call_after_reset();
    test_derivative_dampens_as_measurement_moves();
    test_proportional_only_controller_converges_toward_zero_error();

    std::cout << "All PID tests passed.\n";
    return 0;
}
