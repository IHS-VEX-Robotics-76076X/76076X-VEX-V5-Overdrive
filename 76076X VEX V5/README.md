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

`tests/test_chassis.cpp` and `tests/test_util.cpp` exercise the `Chassis`/
`PID`/`util::` logic on your own machine (no robot required, no ARM
toolchain needed) against a mock PROS API in `include/host/`:

```bash
make HOST_BUILD=1
```

This builds `bin/host_test_chassis` and `bin/host_test_util` and runs both.
It's a separate path from the normal ARM device build (see `Makefile`) -
`HOST_BUILD=1` compiles with your system's own `g++` instead of
cross-compiling for the V5 brain, since the host tests link and run right
here rather than getting uploaded to a robot. This is exactly what CI runs
on every push/PR (see `.github/workflows/host-tests.yml`). To add a new host
test, drop `tests/test_<name>.cpp` in place and add a build rule for it in
the `Makefile` (copy an existing `*_TEST_BIN` pair and rename).

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
  over-current/driver fault (see `pros::MotorGroup::get_faults_all()`), *or*
  if a motor has been fully disconnected (`errno == ENODEV` on the next API
  call - the fault bitfield alone doesn't cover a literally unplugged motor).

All of `drive_distance`/`turn_degrees`/`swing_turn` bail out after
`DRIVE_TIMEOUT_MS`/`TURN_TIMEOUT_MS` (`config.hpp`) instead of hanging
forever if the PID never settles (stall, jam, bad gains).

### Odometry and point-to-point driving

- `set_tracking_wheels(leftWheel, rightWheel, backWheel, wheelDiameterInch)` -
  attaches dedicated, non-powered `pros::Rotation` tracking wheels for
  odometry, so position tracking isn't thrown off by drive-motor wheel slip
  the way reading the drive encoders is. Call **before** `start_odometry()`.
  `leftWheel`/`rightWheel` are the two parallel (forward-measuring) wheels -
  both must be set for tracking wheels to be used at all, or odometry falls
  back to the drive motor encoders. `backWheel` (perpendicular,
  strafe-measuring) is independently optional - pass `nullptr` if you don't
  have one; you still get slip-free forward tracking, just no lateral/strafe
  component (which encoder-only or two-forward-wheel-only odometry can't
  measure at all - useful for catching drift from collisions or scrub during
  turns).
- `start_odometry()` / `stop_odometry()` - spawns/stops a background task
  that dead-reckons `(x, y, heading)` in inches/degrees, from tracking wheels
  if attached or the drivetrain encoders otherwise, plus the IMU for heading
  either way. **Requires an IMU** - `start_odometry()` is a no-op without
  one. Call once, typically from `initialize()`.
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
- `LEFT_TRACKING_WHEEL_PORT` / `RIGHT_TRACKING_WHEEL_PORT` /
  `BACK_TRACKING_WHEEL_PORT` / `TRACKING_WHEEL_DIAMETER_INCH` - tracking
  wheel odometry ports/geometry (see `Chassis::set_tracking_wheels()` above).
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

- `follow_path`/`drive_to_point` are basic go-to-point following, not a true
  curvature-based pure pursuit path follower.
- Odometry and `drive_to_point` require an IMU; there's no encoder-only
  (differential-drive) heading fallback - tracking wheels replace the
  drivetrain encoders for translation, not the IMU for heading.
- No holonomic/mecanum drivetrain support - this is a tank/differential
  drivetrain library throughout (opcontrol mixing, odometry kinematics, and
  `drive`/`drive_forward` all assume it).
