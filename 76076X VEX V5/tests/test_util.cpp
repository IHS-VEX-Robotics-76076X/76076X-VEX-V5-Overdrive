#include <iostream>
#include <cassert>
#include <cmath>
#include <set>

#include "util.hpp"

static void test_clamp() {
    std::cout << "[test] util::clamp\n";

    assert(util::clamp(5.0, 0.0, 10.0) == 5.0);   // inside range: unchanged
    assert(util::clamp(-5.0, 0.0, 10.0) == 0.0);  // below min: clamped up
    assert(util::clamp(15.0, 0.0, 10.0) == 10.0); // above max: clamped down
    assert(util::clamp(0.0, 0.0, 10.0) == 0.0);   // exactly at min: unchanged
    assert(util::clamp(10.0, 0.0, 10.0) == 10.0); // exactly at max: unchanged
    assert(util::clamp(-127.0, -127.0, 127.0) == -127.0);
    assert(util::clamp(200.0, -127.0, 127.0) == 127.0);   // e.g. arcade-mix overshoot
    assert(util::clamp(-200.0, -127.0, 127.0) == -127.0);
}

static void test_sgn() {
    std::cout << "[test] util::sgn\n";

    assert(util::sgn(5.0) == 1);
    assert(util::sgn(0.001) == 1);
    assert(util::sgn(-5.0) == -1);
    assert(util::sgn(-0.001) == -1);
    assert(util::sgn(0.0) == 0);
}

static void test_deadband() {
    std::cout << "[test] util::deadband\n";

    // Default threshold is 5: values with |value| < 5 are zeroed.
    assert(util::deadband(0) == 0);
    assert(util::deadband(3) == 0);
    assert(util::deadband(-3) == 0);
    assert(util::deadband(5) == 5);   // boundary: strictly less-than, so == threshold passes through
    assert(util::deadband(-5) == -5);
    assert(util::deadband(127) == 127);
    assert(util::deadband(-127) == -127);

    // custom threshold
    assert(util::deadband(15, 20) == 0);
    assert(util::deadband(25, 20) == 25);
}

static void test_rand_range() {
    std::cout << "[test] util::randRange\n";

    util::fun(); // seeds the RNG

    // Invalid range (max <= min) returns min instead of dividing by zero or
    // producing an out-of-order range.
    assert(util::randRange(10, 10) == 10);
    assert(util::randRange(10, 5) == 10);

    // Bounds hold across many draws, and a small range should hit more than
    // one distinct value (not just always returning the same number).
    std::set<int> seen;
    for (int i = 0; i < 500; i++) {
        int value = util::randRange(1, 6);
        assert(value >= 1 && value <= 6);
        seen.insert(value);
    }
    std::cout << "  distinct values seen in [1,6] over 500 draws: " << seen.size() << "\n";
    assert(seen.size() > 1);
}

int main() {
    std::cout << "Host test: util\n";

    test_clamp();
    test_sgn();
    test_deadband();
    test_rand_range();

    std::cout << "All util tests passed.\n";
    return 0;
}
