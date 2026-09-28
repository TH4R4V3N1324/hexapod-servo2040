# Hexapod Servo2040

Firmware for the Raspberry Pi Pico-powered Servo 2040 controller used by a six-legged hexapod robot. The project runs on Pimoroni's Servo 2040 board, drives all 18 leg servos, exchanges packed control and telemetry packets with an ESP32 over I²C, and implements gait sequencing, inverse kinematics, trajectory generation, leg calibration, current sensing, and ground-contact switches.

## Features

- 18-channel servo control through Pimoroni Servo 2040
- Six legs with three joints per leg: coxa, femur, and tibia
- Forward and inverse kinematics
- Normal walking and strafing modes
- Tripod, ripple, and wave gaits
- Smooth startup, shutdown, and return-to-start sequences
- Straight, arc, and Bézier foot trajectories
- Body-frame and leg-frame coordinate transforms
- Joystick-driven translation and rotation
- Swing/stance trajectory generation with collision-distance checks
- Per-leg, per-joint calibration offsets
- Calibration persistence in RP2040 flash
- Leg-ground contact switches
- Servo current measurement through the Servo 2040 analog multiplexer
- I²C master communication with an ESP32
- Raspberry Pi Pico SDK and CMake build system

## Hardware and software stack

- **MCU:** Raspberry Pi RP2040
- **Board:** Pimoroni Servo 2040
- **Framework:** Raspberry Pi Pico SDK
- **Build system:** CMake
- **Language:** C++17 with C and assembly enabled by the Pico SDK project
- **Servo driver:** Pimoroni `servo2040` library
- **I/O libraries:** `analogmux`, `analog`, `button`, `hardware_i2c`
- **Communication:** I²C at 100 kHz
- **External peer:** ESP32 at I²C address `0x08`

The project is configured for the Pico board target and requires Raspberry Pi Pico SDK 1.4.0 or newer. The checked-in CMake configuration selects SDK version `1.5.1`, toolchain version `13_2_Rel1`, and picotool version `1.5.1` for the Pico VS Code workflow.

## Repository layout

```text
CMakeLists.txt             Top-level Pico SDK build configuration
pico_sdk_import.cmake      Pico SDK import helper

includes/
  Animation.h              Gait sequencing and motion-state interface
  Calculate.h              Kinematics, transforms, and trajectory generation
  ConfigManager.h          Flash-backed joint calibration interface
  DataPacket.h             Packed packets and I²C declarations
  Hexapod.h                Main application include aggregation
  Move.h                   Servo, switch, current, and leg movement interface

src/
  Hexapod.cpp              Application entry point and command/state FSMs
  Animation.cpp            Gaits, modes, startup, and walking behavior
  Calculate.cpp             Forward/inverse kinematics and trajectories
  ConfigManager.cpp        RP2040 flash calibration storage
  DataPacket.cpp           I²C packet exchange and command tracking
  Move.cpp                 Servo2040 output, switches, and current sensing

external/
  CMakeLists.txt           External dependency entry point
  pimoroni-pico/           Pimoroni Pico libraries and Servo 2040 support
```

## How it fits together

`src/Hexapod.cpp` initializes USB stdio, I²C, calibration data, and the initial body height. It then runs a one-millisecond control loop that reads a `ControlPacket` from the ESP32, handles newly received commands, sends the current `HexPacket` back, and executes the active motion mode.

The `Animation` class chooses the current gait phase and generates swing and stance trajectories. `Calculate` converts between body and leg coordinate frames and calculates joint angles. `Move` applies calibration offsets and writes the calculated angles to the 18-channel Servo 2040 servo cluster while also exposing leg-contact and current-sensing data.

```text
ESP32 controller
      │
      │ I²C, address 0x08
      ▼
DataPacket.cpp
      │
      ├── ReadInputData()
      ├── CommandChanged()
      └── SendHexData()
             │
             ▼
      Hexapod.cpp
       ├── CommandFSM()
       └── StateFSM()
             │
             ▼
        Animation
             │
             ▼
          Move
       ├── Calculate
       ├── ConfigManager
       └── Servo 2040 cluster
```

## Packet protocol

The packet definitions in `includes/DataPacket.h` are packed with `#pragma pack(push, 1)` and must remain byte-compatible with the ESP32 firmware.

### I²C configuration

```cpp
#define ESP32_SLAVE_ADDR 0x08
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21
```

`InitI2C()` configures `i2c0` at 100 kHz and enables pull-ups on GPIO 20 and GPIO 21.

### ControlPacket

```cpp
struct ControlPacket {
    int16_t joystick1X;
    int16_t joystick1Y;
    int16_t joystick2X;
    int16_t joystick2Y;
    int16_t currentHeight;
    Command command;
    int16_t commandArgs[3];
};
```

### HexPacket

```cpp
struct HexPacket {
    int16_t legConfigs[3];
    int16_t currentHeight;
    int8_t currentPhase;
    Gait currentGait;
    Mode currentMode;
};
```

### Commands

```text
CMD_NONE
CMD_SET_GAIT
CMD_SET_MODE
CMD_SET_CONFIG
CMD_HOME_STANCE
CMD_REQUEST_CONFIG
```

`CommandChanged()` compares the current command and its three arguments with the previous values, preventing the same command from being repeatedly applied while the packet remains unchanged.

## Motion control

### Gaits

`Animation::GetLegConfig()` defines the following phase groupings:

| Gait | Phase groups |
| --- | --- |
| Tripod | `{1, 3, 5}` and `{2, 4, 6}` |
| Ripple | `{3, 6}`, `{2, 4}`, and `{1, 5}` |
| Wave | `{3}`, `{2}`, `{1}`, `{4}`, `{5}`, and `{6}` |

Gait changes are queued through `SetGait()` and applied after the robot completes its return-to-start sequence.

### Modes

The state machine in `StateFSM()` supports:

- `MODE_NORMAL` — forward/backward walking with turning
- `MODE_STRAFE` — lateral movement with rotation
- `MODE_TILT` — reserved but currently has no active implementation
- `MODE_CONFIG` — moves the legs toward the configuration pose

### Trajectories

`Calculate` provides three trajectory generators:

- `GenerateStraightTrajectory()` for linear interpolation
- `GenerateArcTrajectory()` for sinusoidal foot lift
- `GenerateBezierTrajectory()` for elevated swing paths

Swing legs use Bézier trajectories. Stance legs use straight trajectories. `Animation::GenerateTrajectories()` converts targets into body coordinates, checks that swing and stance targets remain at least 50 mm apart, and converts the adjusted positions back into each leg's coordinate frame.

### Kinematics

`Calculate::angle()` implements inverse kinematics for the coxa, femur, and tibia joints. `Calculate::position()` performs forward kinematics. The calculation layer also handles mirrored legs, per-leg translation offsets, rotation angles, joystick dead zones, and stride scaling.

The physical link lengths and leg configuration are declared in `includes/Calculate.h`. Update those values when the mechanical geometry changes.

## Servo 2040 output

The `Move` class creates a Servo 2040 cluster covering all 18 servo channels:

```cpp
const uint START_PIN = servo::servo2040::SERVO_1;
const uint END_PIN = servo::servo2040::SERVO_18;
```

The leg-to-servo mapping is:

```text
Leg 1: servos 0, 1, 2
Leg 2: servos 3, 4, 5
Leg 3: servos 6, 7, 8
Leg 4: servos 9, 10, 11
Leg 5: servos 12, 13, 14
Leg 6: servos 15, 16, 17
```

`Move::Position()` performs the following steps:

1. Calculate joint angles with inverse kinematics.
2. Apply the saved calibration offsets.
3. Write coxa, femur, and tibia angles to the Servo 2040 cluster.
4. Update the cached leg position and joint angles.

`Move::Deactivate()` disables the three servo channels associated with a leg.

## Ground-contact switches and current sensing

Each leg is assigned a Servo 2040 analog-multiplexer address:

```text
Leg 1: SENSOR_1_ADDR
Leg 2: SENSOR_2_ADDR
Leg 3: SENSOR_3_ADDR
Leg 4: SENSOR_4_ADDR
Leg 5: SENSOR_5_ADDR
Leg 6: SENSOR_6_ADDR
```

`SetupSwitches()` configures pull-down resistors for the leg switches. `GetSwitchStatus()` reads one switch, while `AllLegsGrounded()` returns true only when all six switches report contact.

Current is measured through the Servo 2040 shared ADC and current-sense channel:

```cpp
mux.select(servo::servo2040::CURRENT_SENSE_ADDR);
float current = cur_adc.read_current();
```

## Calibration storage

`ConfigManager` stores six legs × three joints of `int16_t` offsets in a reserved 4 KiB RP2040 flash sector.

Offsets are clamped to `-60` through `60` before being saved:

```cpp
configManager.SetLegConfig(legNum, joint, offset);
```

The firmware loads offsets during startup and applies them in `Calculate::ApplyOffsets()` before servo output.

> Confirm the configured flash offset in `includes/ConfigManager.h` does not overlap the application image or other persistent data before changing the build layout.

## Building

### Prerequisites

Install:

- Git
- CMake 3.13 or newer
- Ninja or Make
- ARM GCC toolchain for RP2040
- Raspberry Pi Pico SDK 1.4.0 or newer
- Picotool
- A USB cable for the Servo 2040

The project can also be opened with the Raspberry Pi Pico VS Code extension. The top-level `CMakeLists.txt` expects the Pico SDK to be discoverable through the normal SDK environment, CMake cache, or the generated VS Code integration.

### Configure and build

From the repository root:

```bash
mkdir -p build
cd build
cmake .. -DPICO_BOARD=pico -G Ninja
ninja
```

The build creates the `Hexapod` target and generates the standard Pico output artifacts, including UF2 and binary outputs through `pico_add_extra_outputs()`.

With Make instead of Ninja:

```bash
mkdir -p build
cd build
cmake .. -DPICO_BOARD=pico
cmake --build .
```

### Flash the firmware

Put the Servo 2040 into USB mass-storage boot mode, then copy the generated UF2 file to the mounted `RPI-RP2` drive:

```bash
cp build/Hexapod.uf2 /media/$USER/RPI-RP2/
```

The exact mount path varies by operating system. Alternatively, use the Raspberry Pi Pico VS Code extension or picotool according to your board setup.

### USB serial output

The project disables UART stdio and enables USB stdio:

```cmake
pico_enable_stdio_uart(Hexapod 0)
pico_enable_stdio_usb(Hexapod 1)
```

After flashing, connect to the Pico USB serial device to view diagnostic output such as calibration updates and I²C read errors.

## First-boot and safety checklist

This firmware directly controls a walking robot. Before applying servo power:

1. Support the robot so its legs cannot unexpectedly contact the ground.
2. Verify the Servo 2040 board and servo power wiring.
3. Confirm each servo is connected to the intended channel.
4. Confirm leg numbering and mirrored-leg configuration.
5. Confirm the ESP32 peer uses the same packed packet definitions.
6. Verify the I²C address and pins: `0x08`, SDA `20`, SCL `21`.
7. Check the calibration flash offset before flashing a new build.
8. Test one leg and one motion mode at a time.
9. Confirm that all calibration offsets are within safe mechanical limits.

At startup, `main()` waits five seconds and then runs `Animation::Startup()`. The first startup sequence moves each leg through a shutdown/deactivation step before interpolating from the home position to the configured start position.

## Troubleshooting

### CMake cannot find the Pico SDK

- Set `PICO_SDK_PATH` to a valid Pico SDK checkout.
- Confirm that the SDK is version 1.4.0 or newer.
- Remove and recreate the build directory after changing the SDK path.
- If using VS Code, verify the Pico extension's SDK and toolchain settings.

### The build cannot find Servo 2040 libraries

- Confirm that `external/pimoroni-pico` is present and complete.
- Check that `external/CMakeLists.txt` adds the Pimoroni subdirectory.
- Verify that the linked targets include `servo2040`, `analogmux`, `analog`, and `button`.

### I²C packets are not exchanged

- Confirm that the ESP32 is configured as address `0x08`.
- Check SDA/SCL wiring and shared ground.
- Verify GPIO 20 and GPIO 21 are available on the hardware configuration.
- Confirm that both sides use identical packed `ControlPacket` and `HexPacket` layouts.
- Inspect USB serial output for `ReadInputData()` byte-count errors.

### Servos do not move correctly

- Check the leg-to-servo mapping in `includes/Move.h`.
- Verify mechanical servo orientation and mirrored-leg settings.
- Confirm that calibration offsets were loaded correctly.
- Check the Servo 2040 supply voltage and current capacity.
- Test `Move::Angles()` with the robot supported safely.

### Calibration values are invalid after flashing

- Confirm the flash sector reserved by `FLASH_TARGET_OFFSET` is not part of the application image.
- Erasing or reprogramming the reserved sector resets stored offsets.
- Reapply calibration values using `CMD_SET_CONFIG` after a deliberate reset.

## Development notes

- The top-level executable is `Hexapod`, defined in `src/Hexapod.cpp`.
- USB stdio is enabled; UART stdio is disabled.
- The main loop polls I²C, sends telemetry, runs the command FSM, and advances the state FSM every millisecond.
- `MODE_TILT` is defined but currently does nothing.
- Motion trajectories commonly use a resolution of 50 points and a lift height of 50 mm.
- Invalid trajectory resolutions cause `Generate*Trajectory()` to terminate the process.
- External command values are used as leg, joint, gait, and mode indexes; validate controller input before using this firmware in a safety-critical deployment.
- The external Pimoroni Pico tree is included directly under `external/pimoroni-pico`.
- No top-level license file is currently present.

## Contributing

1. Create a feature branch.
2. Keep packet layouts compatible with the ESP32 firmware.
3. Build with CMake before submitting changes.
4. Test with the robot mechanically supported.
5. Document any changes to pin mappings, servo order, I²C settings, gait behavior, or flash layout.

## License

No top-level license file is currently included. Unless a license is added, the project should be treated as all rights reserved. The vendored Pimoroni Pico code and its individual components may have separate licenses; consult the relevant files under `external/pimoroni-pico` before redistributing them.
