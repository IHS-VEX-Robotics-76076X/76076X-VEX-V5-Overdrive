#pragma once

#include "api.h" // for pros::v5::MotorGears, used by the cartridge settings below

#include <array>
#include <cstdint>
#include <cmath>

// ---------------------------------------------------------------------------
// AUTONOMOUS ON / OFF
// ---------------------------------------------------------------------------
//
// While this is false, the robot does NOTHING during the autonomous period.
// It just sits still until driver control starts. The routines in
// autonomous.cpp are never called, and the LCD auton selector is ignored.
//
// This is the right setting while the robot is still being built and tested,
// because the routines in autonomous.cpp are only placeholders - they drive
// blindly forward for a second with no idea what is in front of them.
//
// Flip this to true once real routines are written and tested. Nothing else
// needs changing; the selector and all the routines are still wired up and
// waiting.
constexpr bool AUTON_ENABLED = false;

// ---------------------------------------------------------------------------
// DRIVER CONTROL STYLE
// ---------------------------------------------------------------------------
//
//   SPLIT_ARCADE  Left stick Y drives forward/backward.
//                 Right stick X turns left/right.
//                 One job per thumb, which is why most drivers prefer it.
//
//   ARCADE        Left stick does both: Y drives, X turns.
//                 Right stick unused.
//
//   TANK          Left stick Y drives the left wheels, right stick Y drives
//                 the right wheels. Turning means pushing the sticks by
//                 different amounts.
//
// This is a tank DRIVETRAIN either way - the wheels only spin forward and
// backward, so "turn" always means rotating, never sliding sideways.
enum class DriveMode { SPLIT_ARCADE, ARCADE, TANK };
constexpr DriveMode DEFAULT_DRIVE_MODE = DriveMode::SPLIT_ARCADE;

// ---------------------------------------------------------------------------
// MOTOR GEAR CARTRIDGES
// ---------------------------------------------------------------------------
//
// Every V5 motor has a colored plastic gear cartridge inside it. The color
// sets how fast and how strong the motor is, AND how many encoder "ticks" it
// counts per revolution:
//
//     COLOR    SPEED      TICKS PER TURN    NOTES
//     Red      100 RPM    1800              slowest, strongest
//     Green    200 RPM     900              middle
//     Blue     600 RPM     300              fastest, weakest
//
// Ticks matter because drive_distance() converts inches into ticks. If the
// color here doesn't match the real motor, the robot drives the wrong
// distance - off by a clean 3x between blue and green, with nothing to
// indicate why. TICKS_PER_REV below is derived from this automatically, so the
// ONLY thing you ever change is the color.
//
// Our robot:
//   BLUE  - the 4 drivetrain motors and the intake (built for speed)
//   GREEN - the 2 cascade motors (built for lifting)
constexpr auto DRIVE_MOTOR_GEARSET   = pros::v5::MotorGears::blue;
constexpr auto INTAKE_MOTOR_GEARSET  = pros::v5::MotorGears::blue;
constexpr auto CASCADE_MOTOR_GEARSET = pros::v5::MotorGears::green;

// Looks up encoder ticks per motor revolution from the cartridge color, so
// TICKS_PER_REV can never drift out of sync with the cartridge above.
constexpr double ticks_per_rev_for(pros::v5::MotorGears gearset) {
    return gearset == pros::v5::MotorGears::red   ? 1800.0   // 100 RPM
         : gearset == pros::v5::MotorGears::green ?  900.0   // 200 RPM
         :                                           300.0;  // 600 RPM (blue)
}

// ---------------------------------------------------------------------------
// DRIVETRAIN GEOMETRY
// ---------------------------------------------------------------------------
//
// These are what let the code turn "drive 24 inches" into a tick count.

// Width of the driven wheels across the middle, in inches.
constexpr double WHEEL_DIAMETER_INCH = 3.25;

// Gearing between the motor and the wheel.
//   1.0 = motor connects straight to the wheel
//   2.0 = wheel turns twice per motor turn (geared for speed)
//   0.5 = wheel turns half as often as the motor (geared for torque)
constexpr double GEAR_RATIO = 1.0;

// Encoder ticks per motor revolution, from the drivetrain cartridge color.
// Only valid because Chassis puts its motors in MotorUnits::counts - PROS
// defaults to degrees (360 per turn regardless of cartridge).
constexpr double TICKS_PER_REV = ticks_per_rev_for(DRIVE_MOTOR_GEARSET);

// Encoder ticks per inch the robot actually travels. Worked out once here at
// compile time rather than recalculated in every drive function.
//
//   ticks per inch = ticks per motor turn / (gearing * wheel circumference)
//
// One motor turn moves the robot GEAR_RATIO wheel circumferences, so a faster
// gearing means FEWER ticks per inch.
constexpr double TICKS_PER_INCH = TICKS_PER_REV / (GEAR_RATIO * WHEEL_DIAMETER_INCH * M_PI);

// Drive motor ports: 2 motors per side (4x11W = 44W baseline).
// Override R11a caps Subsystem 1 (drivetrain) at 55W and R11b bans PTO/
// differentials off drive motors - a 3-per-side (66W) drive is illegal.
// 4x11W leaves 11W of Subsystem 1 headroom and 44W of total-budget headroom
// under the 88W robot total (R10a) for lift + manipulator.
//
// ABOUT THE SIGNS
//
// All the driving code assumes one thing: a POSITIVE command means FORWARD,
// on both sides. Turning works by sending +power to one side and -power to
// the other, and odometry works by averaging both sides' encoders, so both
// only behave correctly if that assumption holds.
//
// A negative port number tells PROS "this motor is mounted backwards, flip
// it." Whether a side needs that depends purely on how the motors are
// physically bolted to the robot.
//
// MEASURED ON THIS ROBOT: with all four ports positive, a forward command
// drove the RIGHT side forward and the LEFT side backward - the robot spun
// in place. That's the normal mirrored-drivetrain situation: the motors on
// the two sides face each other across the chassis, so the same rotation
// pushes the robot in opposite directions. The left side gets negated.
//
//   Port -1  left front   (reversed)
//   Port -2  left back    (reversed)
//   Port  3  right front
//   Port  4  right back
//
// Why it looked like "all four spin the same way" when tested by hand: they
// DO all spin the same way. But a motor spinning clockwise on the left side
// of the robot and a motor spinning clockwise on the right side face
// opposite directions, so one drives the wheel forward and the other drives
// it backward. Same rotation, opposite travel. Negating one side fixes it.
//
// Negate the port here rather than changing a sign in the driving code.
// Negating the port makes PROS flip what the encoder reports too, which
// keeps drive_distance() and odometry measuring real forward travel instead
// of the two sides cancelling each other out.
//
// HOW TO RE-CHECK AFTER REWIRING: push the left stick forward.
//   - Drives forward         -> correct
//   - Drives backward        -> flip the sign on all four
//   - Spins in place         -> one whole side is backwards, flip that side
//   - Grinds / barely moves  -> a front and back on the same side are
//                               fighting; flip whichever one runs backward
constexpr std::array<std::int8_t, 2> LEFT_DRIVE_PORTS = {-1, -2};
constexpr std::array<std::int8_t, 2> RIGHT_DRIVE_PORTS = {3, 4};

// ---------------------------------------------------------------------------
// MECHANISM PORTS
// ---------------------------------------------------------------------------
//
// 7 motors total on this robot:
//   4 drivetrain (above)
//   2 cascade lift (always move together, so they're one MotorGroup)
//   1 intake

// Cascade lift: 2 motors, one on each side of the lift, driven as one unit
// so they can never fight. One is negated because they face opposite
// directions across the lift - same idea as the drive ports above. If the
// lift stalls or judders instead of rising, the two are fighting: flip the
// sign on one of them.
constexpr std::array<std::int8_t, 2> CASCADE_MOTOR_PORTS = {5, -6};

// Intake.
constexpr std::int8_t INTAKE_MOTOR_PORT = 7;

// ---------------------------------------------------------------------------
// WHICH SENSORS ARE ACTUALLY INSTALLED
// ---------------------------------------------------------------------------
//
// Flip these to true as the sensors physically go on the robot. Everything
// adapts automatically - no other file needs editing.
//
// Neither is fitted yet, so both are false. What that costs you:
//
//   WORKS WITHOUT EITHER SENSOR
//     - all of driver control
//     - drive_distance(), because it counts the drive motors' own encoders
//
//   NEEDS THE IMU
//     - turn_degrees() and swing_turn(), which steer by measured angle
//     - position tracking, and drive_to_point()/follow_path() built on it
//
// Anything unavailable stops the motors and returns instead of running on
// garbage readings, so calling it is safe - it just won't do anything.
//
// The tracking wheel is a pure upgrade rather than a requirement: position
// tracking falls back to the drive encoders without it. Those slip under
// hard acceleration, so the wheel makes tracking more accurate, but nothing
// stops working if it's absent.
constexpr bool HAS_INERTIAL_SENSOR = false;
constexpr bool HAS_TRACKING_WHEEL  = false;

// Sensors.
constexpr std::uint8_t INERTIAL_SENSOR_PORT = 11; // the IMU / gyro

// Motor power budget (V5 Smart Motors: 11W full, 5.5W half).
constexpr int WATT_PER_11W_MOTOR = 11;
constexpr int SUBSYSTEM1_MAX_WATT = 55; // R11a drivetrain cap
constexpr int ROBOT_MAX_WATT = 88;      // R10a robot total cap
constexpr int MECHANISM_MOTOR_COUNT =
    CASCADE_MOTOR_PORTS.size() + 1; // + intake
static_assert((LEFT_DRIVE_PORTS.size() + RIGHT_DRIVE_PORTS.size()) * WATT_PER_11W_MOTOR <= SUBSYSTEM1_MAX_WATT,
    "Illegal drivetrain: Subsystem 1 exceeds 55W (R11a). Use at most 5x11W, e.g. 4x11W.");
static_assert((LEFT_DRIVE_PORTS.size() + RIGHT_DRIVE_PORTS.size() + MECHANISM_MOTOR_COUNT) * WATT_PER_11W_MOTOR <= ROBOT_MAX_WATT,
    "Illegal robot: total exceeds 88W (R10a). Count drive + cascade + intake.");

// ---------------------------------------------------------------------------
// TRACKING WHEEL (ODOMETRY) - we have exactly ONE, facing forward
// ---------------------------------------------------------------------------
//
// A tracking wheel is a small wheel that NO motor drives. It just rolls along
// the ground and reports how far it has spun.
//
// Why bother, when the drive motors already have encoders? Because a powered
// wheel can spin without the robot actually moving - it slips when you
// accelerate hard or shove into something. A free-spinning wheel doesn't lie.
//
// One forward-facing wheel is enough for full position tracking: the wheel
// says HOW FAR we went, the IMU says WHICH WAY we were pointed. Combine them
// every tick and you can add up the robot's path.
//
// What one wheel can't do is measure sideways drift. If the robot gets shoved
// sideways, odometry won't notice. That needs a second wheel mounted
// perpendicular, which we don't have.
//
// If this sensor isn't plugged in, odometry automatically falls back to the
// drive motor encoders. Position tracking still works, just with more drift.
//
// Port uses the same negative-for-reversed convention as motor ports. If the
// tracked distance goes DOWN when the robot drives forward, negate this.
constexpr std::int8_t TRACKING_WHEEL_PORT = 12;

// Width of the tracking wheel across the middle, in inches. 2.75 is the
// common VEX size.
constexpr double TRACKING_WHEEL_DIAMETER_INCH = 2.75;

// Default PID gains (safe defaults; tune for your robot)
constexpr double DEFAULT_DRIVE_KP = 0.5;
constexpr double DEFAULT_DRIVE_KI = 0.0;
constexpr double DEFAULT_DRIVE_KD = 0.0;

constexpr double DEFAULT_TURN_KP = 1.0;
constexpr double DEFAULT_TURN_KI = 0.0;
constexpr double DEFAULT_TURN_KD = 0.0;

// PID windup guard and settling tolerances. Drive error is in encoder ticks
// (can be thousands) and turn error is in degrees (0-180), so they need very
// different scales - do not share one set of values between the two PIDs.
constexpr double DEFAULT_DRIVE_INTEGRAL_CAP = 5000.0;
constexpr double DEFAULT_DRIVE_SETTLE_ERROR = 30.0;    // ticks
constexpr double DEFAULT_DRIVE_SETTLE_VELOCITY = 5.0;  // ticks/loop

constexpr double DEFAULT_TURN_INTEGRAL_CAP = 20.0;
constexpr double DEFAULT_TURN_SETTLE_ERROR = 2.0;      // degrees
constexpr double DEFAULT_TURN_SETTLE_VELOCITY = 0.5;   // degrees/loop

// Safety timeouts: if a movement's PID loop never settles (stall, jam,
// disconnected motor, bad gains), it bails out after this many ms instead of
// hanging the autonomous/opcontrol task forever.
constexpr int DRIVE_TIMEOUT_MS = 3000;
constexpr int TURN_TIMEOUT_MS = 2000;

// Stall detection: exit early when commanding significant power but making
// no encoder/IMU progress (e.g. pushed into a goal). Saves auton clock vs
// burning the full timeout above. Progress is measured as position delta
// (no extra sensor API needed, works on device and host mock alike).
constexpr int STALL_TIMEOUT_MS = 500;              // no progress for this long -> stalled
constexpr double STALL_DRIVE_PROGRESS_TICKS = 10.0; // min encoder ticks considered progress
constexpr double STALL_TURN_PROGRESS_DEG = 1.0;     // min degrees considered progress
constexpr double STALL_MIN_OUTPUT = 20.0;           // ignore stall when barely commanding power

// Opcontrol shaping: slew limits jerk with tall stacks (tip/descore risk),
// expo gives fine control near center for goal alignment.
constexpr int OPCONTROL_SLEW_PER_LOOP = 10; // max speed-up per 20ms loop (slowing down is instant)
constexpr double OPCONTROL_EXPO_GAIN = 0.4; // 0 = linear, 1 = full cubic blend

// Proportional gain correcting heading drift during drive_distance() using
// the IMU. 0 disables correction (e.g. when no IMU is connected).
constexpr double DEFAULT_HEADING_KP = 1.0;

// Basic motion profiling: caps how much drive_distance()'s output can change
// per 10ms loop (out of the -127..127 motor range), so the drivetrain ramps
// up from a dead stop instead of slamming to full power and slipping.
constexpr double DRIVE_MAX_ACCEL_PER_LOOP = 15.0;
