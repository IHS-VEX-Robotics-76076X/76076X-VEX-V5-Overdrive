Project: 76076X VEX V5

This repository contains PROS-based firmware for a VEX V5 robot.

Building (recommended):
- Install PROS toolchain and CLI per PROS documentation: https://pros.cs.purdue.edu/
- Ensure `arm-none-eabi-gcc` and Newlib headers are available on your PATH.
- From the project root run:

```bash
pros make
```

Notes for portability:
- This project includes small `include/stubs/` fallbacks for minimal C/C++ headers so source parsing and static analysis can work on systems without the full cross-toolchain. These are not replacements for a proper PROS toolchain.
- For actual device builds and linking, install the official PROS toolchain or use the PROS Docker image.

Common pitfalls on macOS:
- If `make` fails complaining about missing `stdint.h` or other headers, ensure an arm-none-eabi toolchain with newlib is installed (Homebrew or PROS installer). Example:

```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc
```

If you'd like, I can:
- Continue making the code more defensive and portable (optional IMU handling, config, safety clamps), or
- Help you install/verify the PROS toolchain on this machine.
