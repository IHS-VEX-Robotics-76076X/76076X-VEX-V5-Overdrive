### 76076X-VEX-V5-Overdrive

### PLEASE WORK ON THE LIBRARY

### How it currently works

## Overview

This library is a PROS based control system for a tank drive VEX V5 robot. It gives you PID controlled driving and turning, dead reckoning odometry using either drive motor encoders or dedicated tracking wheels, point to point navigation, and a host side test suite so you can check your logic on your own computer before ever touching the robot.

## The Chassis class

Chassis wraps two pros::MotorGroup objects (left side and right side), an optional pros::Imu pointer, two PID controllers (one for driving straight, one for turning), and the odometry state.

Basic movement functions

- drive_forward(speed, forward) and drive(leftSpeed, rightSpeed) send raw voltage commands from -127 to 127 directly to the motors. Both are clamped automatically so a bad input never gets sent straight to the motors.
- stop() sets both sides to zero.

Controlled movement functions

- drive_distance(inches) converts inches to encoder ticks using TICKS_PER_INCH, which is computed from TICKS_PER_REV times GEAR_RATIO divided by (WHEEL_DIAMETER_INCH times pi). It reads both motor groups' average position each loop, feeds the error into the drive PID, clamps the output, then ramps it using DRIVE_MAX_ACCEL_PER_LOOP so the robot does not slam to full power and slip. If an IMU is attached, it also applies a small proportional correction (headingKP) to hold the starting heading. It bails out after DRIVE_TIMEOUT_MS if the PID never settles.
- turn_degrees(degrees) reads imu get_rotation(), which is an unbounded angle (it keeps counting past 360 instead of wrapping), adds the requested degrees to get a target angle, then drives the turn PID until settled or until TURN_TIMEOUT_MS runs out. Without an IMU this is a no op, but it still stops the motors first so the robot never keeps moving from a previous command.
- swing_turn(degrees, side) is the same idea but only powers one side. The other side is commanded to zero, and since brake mode is set to HOLD at construction, that side acts as a pivot point instead of coasting.

Fault detection

has_fault() checks get_faults_all() on both motor groups for over temperature, over current, or driver fault bits. It also does a get_position_all() call and checks errno for ENODEV, since a fully unplugged motor does not set any fault bit, it just fails silently on the next API call.

## PID controller

PID (in pid.h) is a standard proportional integral derivative controller. calculate(error, measurement) takes both the error (target minus current value) and the raw measurement itself, because the derivative term is computed from the measurement's rate of change, not the error's rate of change. This avoids a problem called derivative kick, where a sudden change in target (like calling turn_degrees right after reset()) would otherwise look like a huge fake velocity spike to the derivative term. The very first calculate() call after construction or reset() has no previous measurement to compare against, so it reports zero derivative that one time instead of guessing.

isSettled(error) returns true only when both the error and the derivative (rate of change) are small, so the robot does not report "arrived" just because it is passing through the target at speed.

integralCap limits how large the integral term can grow so it does not wind up out of control. settleError and settleVelocity are the thresholds used above. All of these are set per PID instance because a drive PID (error measured in thousands of ticks) and a turn PID (error measured in degrees) need very different numbers.

## Odometry

Odometry is a background task, started with start_odometry(), that runs every 10 milliseconds and keeps a running (x, y, heading) position estimate using dead reckoning. It needs an IMU for heading, so start_odometry() does nothing if no IMU was given.

Two ways to measure movement

- Dedicated tracking wheels (pros::Rotation sensors) if you call set_tracking_wheels() first. These are unpowered wheels that just spin freely, so they are not affected by wheel slip the way a powered drive wheel is. Their readings come back in centidegrees, so the library converts using wheelDiameterInch times pi divided by 36000 to get inches per centidegree.
- If you do not have tracking wheels, it falls back to averaging the drive motor's own encoder positions. This works, but is more prone to drift if the wheels ever slip.

A third optional back tracking wheel, mounted sideways, can measure left and right drift (strafe) that the two forward wheels cannot see at all.

Coordinate system

Heading 0 points along the positive Y axis and increases clockwise, matching imu get_rotation() directly. This is not the usual math convention (0 along positive X, counter clockwise positive), so the position update uses sin and cos in a swapped pattern to stay consistent. Each tick, forward and strafe movement gets rotated by the current heading and added into odomX and odomY.

Reliability details

- Before integrating any reading, the library checks it is not PROS_ERR (from a disconnected Rotation sensor) or non finite (from a disconnected drive motor, which reports PROS_ERR_F, meaning infinity). A bad reading is skipped for that tick instead of being added to the position, so a temporary disconnect does not permanently corrupt the position forever.
- The very first baseline reading is also validated this way (not just later ticks), so a sensor that happens to be disconnected at the exact moment start_odometry() runs cannot poison the starting baseline either.
- odomX, odomY, and odomHeading are all read and written behind a mutex, so a caller reading position from another task never sees a half updated set of values.
- reset_position(x, y, headingDeg) stores the difference between your declared heading and the IMU's raw reading in a separate offset variable, so the heading you set actually sticks instead of getting overwritten by the IMU's own number on the next tick.

## Point to point driving

drive_to_point(x, y) reads the current odometry snapshot, computes the straight line distance and bearing to the target using atan2(dx, dy) (matching the same coordinate convention as above), then calls turn_degrees() followed by drive_distance(). If the target is less than half an inch away, it does nothing instead of turning toward a meaningless bearing. This requires start_odometry() to already be running, otherwise it is a no op since there is no live position to navigate from. follow_path() just calls drive_to_point() once per waypoint in order. This is simple sequential point to point movement, not a curve based path follower.

## Testing without a robot

The include/host folder contains a mock version of the PROS API, so the whole Chassis, PID, and util logic can be compiled and run on your own computer with a normal C++ compiler, no VEX hardware or ARM toolchain required. Running

make HOST_BUILD=1

builds and runs three test binaries, one for chassis logic, one for the small util helpers, and one for the PID math specifically. This same command runs automatically on every push and pull request through GitHub Actions, so a broken change gets caught before it reaches the robot.

## Configuration

Everything you would normally tune lives in config.hpp instead of being scattered through the code. This includes motor ports, wheel diameter, gear ratio, tracking wheel ports and diameter, all PID gains and tolerances, both safety timeouts, the heading correction gain, the drive mode (arcade or tank), and the acceleration limit used in drive_distance.

### pending work

## software to do list:
1. strengthen the library and/or check for bugs
2. organize folders(maybe)

## hardware to do list:
1. Drivetrain motor ports/wheel geometry: `config.hpp`, `LEFT_DRIVE_PORTS`, `RIGHT_DRIVE_PORTS`, `WHEEL_DIAMETER_INCH`, `GEAR_RATIO`, `TICKS_PER_REV`
2. Mechanism motor ports: `config.hpp`, `CASCADE_MOTOR_PORT`, `INTAKE_MOTOR_PORT`, `ARM_TURN_MOTOR_PORT`, `CLAMP_MOTOR_PORT`
3. IMU port: `config.hpp`, `INERTIAL_SENSOR_PORT`
4. Tracking wheel ports/diameter: `config.hpp`, `LEFT_TRACKING_WHEEL_PORT`, `RIGHT_TRACKING_WHEEL_PORT`, `BACK_TRACKING_WHEEL_PORT`, `TRACKING_WHEEL_DIAMETER_INCH` (still placeholder values; remove the back wheel wiring in `main.cpp`/`set_tracking_wheels()` if you don't have one)
5. PID gains and tolerances: `config.hpp`, `DEFAULT_DRIVE_KP/KI/KD`, `DEFAULT_TURN_KP/KI/KD`, `DEFAULT_DRIVE_INTEGRAL_CAP/SETTLE_ERROR/SETTLE_VELOCITY`, `DEFAULT_TURN_*` equivalents, `DEFAULT_HEADING_KP`, `DRIVE_MAX_ACCEL_PER_LOOP`, all currently generic placeholder values, need tuning on the real robot
6. Safety timeouts: `config.hpp`, `DRIVE_TIMEOUT_MS`, `TURN_TIMEOUT_MS` (defaults are reasonable but worth revisiting once real movement speeds are known)
7. Drive mode / control scheme: `config.hpp`, `DEFAULT_DRIVE_MODE` (arcade vs. tank, a driver preference, not yet decided)
8. Real autonomous routines: `src/autonomous.cpp`, `red_close_side()` and `blue_far_side()` are explicit placeholders (drive forward/backward 1 second), need the actual game-strategy routines once decided
9. Opcontrol button mapping for mechanisms: `src/opcontrol.cpp`, cascade/arm/clamp/intake button bindings are a reasonable guess but should be confirmed against actual driver preference/game mechanism design
10. No mecanum/holonomic support: entire library (`chassis.hpp`/`chassis.cpp`, `opcontrol.cpp`), tank drive only by design, would need new hardware plus a rewrite if the robot ever goes holonomic
11. No pneumatics/additional-sensor support (e.g. distance, optical, vision): not present anywhere, add if/when that hardware is added
12. Local ARM toolchain: your machine's `arm-none-eabi-gcc` install is missing newlib headers, so `pros make`/device builds currently fail before even reaching this repo's code, fix is in `README.md`'s "Common pitfalls on macOS" section (`brew install arm-none-eabi-gcc`), this is a machine setup issue, not a code change
