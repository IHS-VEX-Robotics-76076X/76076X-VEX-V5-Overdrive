# 76076X VEX V5

PROS-based firmware for the 76076X VEX V5 competition robot, built around a
small `Chassis` (tank drivetrain + PID + odometry) and `PID` library.

## Building for the robot

- Install the PROS toolchain and CLI per the [PROS documentation](https://pros.cs.purdue.edu/).
- Ensure `arm-none-eabi-gcc` and Newlib headers are on your `PATH`.
- From the project root run:

```bash
pros make
```

### Common pitfalls on macOS

If `make` fails complaining about missing `stdint.h` or other headers, ensure
an arm-none-eabi toolchain with newlib is installed:

```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc
```

## Building and running the host tests

`tests/test_chassis.cpp` exercises the `Chassis`/`PID` logic on your own
machine (no robot required) against a mock PROS API in `include/host/`.

> **Note:** `make HOST_BUILD=1` does not currently work - it still tries to
> link against the ARM firmware `.ld` scripts. Compile the test directly
> with `g++` instead:

```bash
g++ -std=gnu++20 -pthread -Wall -DHOST_BUILD -Iinclude \
    -o /tmp/test_chassis src/chassis.cpp src/util.cpp tests/test_chassis.cpp
/tmp/test_chassis
```

This is exactly what CI runs on every push/PR (see
`.github/workflows/host-tests.yml`).

## Chassis API

`Chassis` (`include/chassis.hpp`) wraps a two-side (left/right) tank
drivetrain plus an optional IMU:

- `drive(leftSpeed, rightSpeed)` / `drive_forward(speed, forward)` / `stop()` -
  direct, unregulated motor control (used by opcontrol).
- `drive_distance(inches)` - drives straight using the drive `PID`, both
  sides' encoders (averaged), IMU heading-hold correction, and a basic
  accel-limited (motion-profiled) ramp. Requires an IMU for heading
  correction, but works without one (just skips the correction).
- `turn_degrees(degrees)` - point turn to a relative angle using the IMU and
  turn `PID`. No-op without an IMU.
- `swing_turn(degrees, Chassis::DriveSide::LEFT|RIGHT)` - turns by driving
  only one side, pivoting on the other (locked) side.
- `has_fault()` - true if any drivetrain motor is reporting an over-temp/
  over-current/driver fault (see `pros::MotorGroup::get_faults_all()`).

All of `drive_distance`/`turn_degrees`/`swing_turn` bail out after
`DRIVE_TIMEOUT_MS`/`TURN_TIMEOUT_MS` (`config.hpp`) instead of hanging
forever if the PID never settles (stall, jam, bad gains).

### Odometry and point-to-point driving

- `start_odometry()` / `stop_odometry()` - spawns/stops a background task
  that dead-reckons `(x, y, heading)` in inches/degrees from the drivetrain
  encoders and the IMU. **Requires an IMU** - `start_odometry()` is a no-op
  without one. Call once, typically from `initialize()`.
- `get_x()` / `get_y()` / `get_heading()` / `reset_position(x, y, headingDeg)`.
  Uses a compass-style convention matching `imu->get_rotation()` directly:
  heading 0 = +Y axis, clockwise-positive (the same direction
  `turn_degrees(+degrees)` actually turns the robot) - not the standard math
  convention (0 = +X axis, counter-clockwise-positive).
- `drive_to_point(x, y)` - turns to face `(x, y)` then drives straight to it,
  using odometry feedback. This is sequential point-to-point ("go to point")
  following, **not** curvature-based pure pursuit.
- `follow_path({{x1,y1}, {x2,y2}, ...})` - calls `drive_to_point` for each
  waypoint in order.

`drive_to_point`'s bearing math is internally consistent with odometry (both
use the same convention above), but which physical direction on the field
counts as "+X"/"+Y" still depends on how the IMU happens to be mounted/
oriented - verify on-robot before relying on it in a match.

## PID

`PID` (`include/pid.h`) is a standard kP/kI/kD controller. `integralCap`,
`settleError`, and `settleVelocity` are constructor parameters (not shared
globals) because a drive PID (error in encoder ticks, can be thousands) and a
turn PID (error in degrees, 0-180) need very different scales -
`config.hpp`'s `DEFAULT_DRIVE_*`/`DEFAULT_TURN_*` constants set sane defaults
for each. `isSettled()` requires both the error and its rate of change to be
small, so a fast pass through the target isn't mistaken for having arrived.

## Configuration (`include/config.hpp`)

All of the following live in one place - change them there, not at the call
sites in `main.cpp`:

- `WHEEL_DIAMETER_INCH`, `GEAR_RATIO`, `TICKS_PER_REV` - drivetrain geometry.
- `LEFT_DRIVE_PORTS` / `RIGHT_DRIVE_PORTS` - drivetrain motor ports.
- `CASCADE_MOTOR_PORT` / `INTAKE_MOTOR_PORT` / `ARM_TURN_MOTOR_PORT` /
  `CLAMP_MOTOR_PORT` / `INERTIAL_SENSOR_PORT` - mechanism/sensor ports.
- `DEFAULT_DRIVE_MODE` (`DriveMode::ARCADE` or `::TANK`) - opcontrol stick
  layout. ARCADE is single-stick (left Y forward, left X turn); TANK is two
  sticks (left Y / right Y per side). This is a tank drivetrain, not
  mecanum, so "arcade" here still means turning, not strafing.
- `DEFAULT_DRIVE_KP/KI/KD`, `DEFAULT_TURN_KP/KI/KD` - PID gains.
- `DEFAULT_DRIVE_INTEGRAL_CAP/SETTLE_ERROR/SETTLE_VELOCITY`,
  `DEFAULT_TURN_INTEGRAL_CAP/SETTLE_ERROR/SETTLE_VELOCITY` - PID tolerances.
- `DRIVE_TIMEOUT_MS` / `TURN_TIMEOUT_MS` - safety timeouts.
- `DEFAULT_HEADING_KP` - heading-hold correction gain for `drive_distance`.
- `DRIVE_MAX_ACCEL_PER_LOOP` - motion-profiling accel limit for `drive_distance`.

## On the LCD

- **Left/right buttons** (during `competition_initialize`, i.e. before a
  match starts): pick the autonomous routine (red close side / blue far
  side). Shown on line 3.
- **Center button**: toggles a live status readout on line 2 - robot battery
  %, controller connection, and any motor faults.

## Known limitations

- `make HOST_BUILD=1` doesn't work (see above) - use the direct `g++`
  command or `.github/workflows/host-tests.yml` as a reference.
- `follow_path`/`drive_to_point` are basic go-to-point following, not a true
  curvature-based pure pursuit path follower.
- Odometry and `drive_to_point` require an IMU; there's no encoder-only
  (differential-drive) heading fallback.
