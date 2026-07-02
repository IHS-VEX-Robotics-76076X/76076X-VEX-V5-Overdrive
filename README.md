# 76076X-VEX-V5-Overdrive
### PLEASE WORK ON THE LIBRARY

software to do list:
1. make the library
2. organize folders

hardware to do list:
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
