#### 76076X VEX V5

#### PLEASE WORK ON THE LIBRARY

---

## What this is

Code for our VEX V5 competition robot, written with PROS.

It handles driving the robot by hand during driver control, driving exact
distances and angles by itself during autonomous, and keeping track of where
the robot is on the field the whole time.

## The robot

8 motors total, plus 2 sensors.

| Part | Motors | Cartridge | Notes |
|---|---|---|---|
| Drivetrain | 4 (2 per side) | Blue (600 RPM) | tank drive |
| Cascade lift | 2 | Green (200 RPM) | move together as one |
| Intake | 1 | Blue (600 RPM) | |
| Arm | 1 | Green (200 RPM) | |
| IMU (gyro) | sensor | | tells us which way we face |
| Tracking wheel | sensor | | tells us how far we travel |

---

## Which file does what

Start here when you are looking for something.

### Files you will actually edit

| File | What it is for |
|---|---|
| **`include/config.hpp`** | **Every number about the robot.** Port numbers, motor colors, wheel sizes, PID tuning, timeouts. If you are changing hardware or tuning, you are editing this file and probably nothing else. |
| **`src/autonomous.cpp`** | The autonomous routines. Currently placeholders that just drive forward and back. |
| **`src/opcontrol.cpp`** | Driver control. Which joystick drives, which button runs the intake, and so on. |

### The library itself

| File | What it is for |
|---|---|
| **`include/chassis.hpp`** | The list of everything the drivetrain can do, with an explanation of each. **Read this first** to learn what functions are available. |
| **`src/chassis.cpp`** | The actual code behind those functions. Driving, turning, and position tracking all live here. |
| **`include/pid.h`** | The PID controller, which is what lets the robot hit an exact distance or angle instead of guessing. |
| **`include/util.hpp`** and **`src/util.cpp`** | Small math helpers: clamp, deadband, sign, random. |
| **`src/main.cpp`** | Startup. Creates all the motors and sensors, calibrates the IMU, and decides what runs when. |

### Support files

| File | What it is for |
|---|---|
| `include/host/pros_mock.hpp` | A fake version of the VEX API so tests can run on a laptop with no robot attached. |
| `tests/test_chassis.cpp` | Tests for driving, turning, and position tracking. |
| `tests/test_pid.cpp` | Tests for the PID math. |
| `tests/test_util.cpp` | Tests for the helper functions. |
| `Makefile` | Build settings. |
| `include/pros/`, `include/liblvgl/` | The PROS library itself. Do not edit these. |

---

## What runs when

The V5 brain calls these automatically, in this order:

1. **`initialize()`** in `main.cpp` runs once at power-on. Calibrates the IMU (takes about 2 seconds), sets brake modes, and starts position tracking.
2. **`competition_initialize()`** in `main.cpp` runs while waiting for the match. Left and right LCD buttons pick which autonomous routine to use.
3. **`autonomous()`** in `main.cpp` runs the chosen routine from `autonomous.cpp`. 15 seconds, nobody driving.
4. **`opcontrol()`** in `opcontrol.cpp` runs for the rest of the match. Loops forever reading the controller.

---

## How the robot knows where it is

The robot keeps a running guess of its position, updated 100 times a second
in the background. This is called odometry. It needs two things:

- **How far it moved** comes from the tracking wheel. This is a small wheel
  that no motor drives. It just rolls along and reports how far it spun.
  Why not use the drive motors? Because a powered wheel can spin without the
  robot moving. It slips when you accelerate hard or hit something. A
  free-spinning wheel does not lie.
- **Which way it faces** comes from the IMU.

Combine those every few milliseconds and you can add up the robot's path
step by step.

If the tracking wheel is not plugged in, the code automatically falls back to
reading the drive motors instead. Position tracking still works, it just
drifts more over a match.

**Odometry needs the IMU.** Without one, `start_odometry()` does nothing at all.

### Directions

Position is `(x, y)` in inches, heading is in degrees.

- Heading `0` means facing along `+Y`
- Heading `90` means facing along `+X`
- Turning right (clockwise) makes the heading go **up**

This is compass style, like a real compass where north is 0 and east is 90.
It is not the convention from math class, where 0 points along `+X` and
angles increase counter-clockwise. We use compass style because it matches
what the IMU reports directly, so nothing has to be converted.

Which physical corner of the field counts as `+X` depends on how the IMU is
mounted and where the robot starts. Check this on the real robot before
trusting it in a match.

---

## Using the chassis

Everything below is a method on `myRobot`, which is created in `main.cpp`.

### Driving by hand

Power ranges from `-127` (full reverse) to `127` (full forward). Out of range
numbers are clamped automatically.

```cpp
myRobot.drive(leftPower, rightPower);   // each side separately
myRobot.drive_forward(power, forward);  // both sides together
myRobot.stop();
```

### Driving exact amounts

These block until the robot arrives, then stop the motors.

```cpp
myRobot.drive_distance(24);                          // 24 inches forward
myRobot.drive_distance(-12);                         // 12 inches backward
myRobot.turn_degrees(90);                            // 90 degrees right
myRobot.swing_turn(45, Chassis::DriveSide::LEFT);    // pivot on left wheels
```

If one of these never reaches its target, because something jammed or a motor
stalled or a PID gain is wrong, it gives up after the timeout in `config.hpp`
instead of freezing for the rest of the match.

### Position

```cpp
myRobot.reset_position(0, 0, 0);   // declare where you are starting
double x = myRobot.get_x();
double y = myRobot.get_y();
double h = myRobot.get_heading();
```

### Driving to a spot

```cpp
myRobot.drive_to_point(24, 36);                    // turn to face it, then go
myRobot.follow_path({{24, 0}, {24, 24}, {0, 24}}); // several spots in order
```

Both need `start_odometry()` to already be running, since they need to know
where the robot currently is. This is simple "turn, then go" movement. It
does not follow smooth curves and it will not avoid obstacles.

---

## Controls

| Input | Does |
|---|---|
| Left stick | Drive. Forward to move, sideways to turn. |
| L1 / L2 | Cascade lift up / down |
| R1 / R2 | Arm up / down |
| X | Intake |
| LCD left / right | Pick the autonomous routine (before the match) |
| LCD center | Toggle the status readout on the brain screen |

Driver control is single-stick arcade by default. To switch to two-stick tank,
change `DEFAULT_DRIVE_MODE` in `config.hpp`.

---

## Building

### Onto the robot

```bash
pros make
```

You need the PROS toolchain installed, with `arm-none-eabi-gcc` on your
`PATH`. See the [PROS docs](https://pros.cs.purdue.edu/).

If the build fails complaining about a missing `stdint.h` or similar, your
ARM toolchain is incomplete. On macOS:

```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc
```

### Testing on your laptop, with no robot

```bash
make HOST_BUILD=1
```

This compiles the chassis, PID, and helper code against a fake VEX API
(`include/host/pros_mock.hpp`) using your computer's normal C++ compiler, then
runs all the tests. No robot and no ARM toolchain needed.

Use this before uploading anything. It catches broken math and logic errors
in seconds instead of during a match. GitHub also runs it automatically on
every push and pull request.

To add a new test file, drop `tests/test_<name>.cpp` in place and copy an
existing `*_TEST_BIN` pair in the `Makefile`.

---

## Known limitations

- `drive_to_point` and `follow_path` turn and then drive straight. They do not
  follow smooth curves.
- Odometry and `drive_to_point` need the IMU. There is no fallback if it is
  missing or broken.
- Tank drive only. The robot cannot slide sideways, and the code assumes that
  everywhere.
- Only one tracking wheel, facing forward, so sideways drift cannot be
  detected. If the robot gets shoved sideways, odometry will not notice.

---

## Still to do

### Software

1. Strengthen the library and keep checking for bugs
2. Organize folders (maybe?)

### Needs the real robot

1. **Port numbers** in `config.hpp`, all currently placeholders: `LEFT_DRIVE_PORTS`, `RIGHT_DRIVE_PORTS`, `CASCADE_MOTOR_PORTS`, `INTAKE_MOTOR_PORT`, `ARM_MOTOR_PORT`, `INERTIAL_SENSOR_PORT`, `TRACKING_WHEEL_PORT`
2. **Wheel sizes and gearing** in `config.hpp`: `WHEEL_DIAMETER_INCH`, `GEAR_RATIO`, `TRACKING_WHEEL_DIAMETER_INCH`
3. **PID tuning** in `config.hpp`, the biggest job, needs a real robot to test against: all the `DEFAULT_DRIVE_*` and `DEFAULT_TURN_*` gains, plus `DEFAULT_HEADING_KP` and `DRIVE_MAX_ACCEL_PER_LOOP`
4. **Timeouts** in `config.hpp` once real speeds are known: `DRIVE_TIMEOUT_MS`, `TURN_TIMEOUT_MS`
5. **Drive mode** in `config.hpp`, a driver preference: `DEFAULT_DRIVE_MODE`
6. **Real autonomous routines** in `src/autonomous.cpp`, currently placeholders
7. **Button mapping** in `src/opcontrol.cpp`, confirm with whoever is driving
8. No pneumatics or extra sensors (distance, optical, vision) are wired up. Add if that hardware goes on the robot.
