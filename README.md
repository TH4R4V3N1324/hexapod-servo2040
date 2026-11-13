# Hexapod Servo2040

Firmware for the Raspberry Pi Pico-powered Servo 2040 controller used to run the Explorer hexapod: the embedded counterpart of the [hexapod-ros2](https://github.com/TH4R4V3N1324/hexapod-ros2) stack, with the same geometry and control scheme. The project runs on Pimoroni's Servo 2040 board, drives all 18 leg servos and exchanges packed control and telemetry packets with an ESP32 over I²C. The gait itself (kinematics, gait sequencing, trajectories, sitting, tilt mode) comes from [hexapod-gait-core](https://github.com/TH4R4V3N1324/hexapod-gait-core), the same library the [hexapod-ros2](https://github.com/TH4R4V3N1324/hexapod-ros2) stack runs, so both drive the robot the same way.

## Features

- Gait from hexapod-gait-core's `GaitEngine`, shared with hexapod-ros2
- Explorer geometry (`RobotGeometry::explorer()`), matching the hexapod-ros2 URDF
- 18-channel servo control through Pimoroni Servo 2040
- Normal walking (forward and turning), strafing, and tilt mode (body shift, roll and pitch)
- Tripod, ripple, and wave gaits, changed at the next step
- Startup into a seated pose, then standing up; sit and stand on command
- Body leveling from an ISM330DHCX IMU while walking and in tilt mode, as on hexapod-ros2
- Body height changes that glide rather than jump
- Feet return to their stance by themselves when the sticks are idle
- Per-leg, per-joint calibration offsets stored in RP2040 flash, and a calibration pose (`MODE_CONFIG`)
- Leg-ground contact switches and servo current measurement
- I²C master communication with an ESP32; the sticks count as centred if it stops answering
- Raspberry Pi Pico SDK and CMake build system

## Hardware and software stack

- **MCU:** Raspberry Pi RP2040
- **Board:** Pimoroni Servo 2040
- **Framework:** Raspberry Pi Pico SDK
- **Build system:** CMake
- **Language:** C++17 with C and assembly enabled by the Pico SDK project
- **Gait:** hexapod-gait-core v0.3.0 (git submodule, float `Scalar`, bundled Eigen)
- **Servo driver:** Pimoroni `servo2040` library
- **I/O libraries:** `analogmux`, `analog`, `button`, `hardware_i2c`
- **Communication:** I²C at 100 kHz
- **External peer:** ESP32 at I²C address `0x08`
- **IMU:** ISM330DHCX (or ISM330DLC) at I²C address `0x6A`, on the same bus

The project is configured for the Pico board target and requires Raspberry Pi Pico SDK 2.0.0 or newer (the Pimoroni libraries use SDK 2 APIs) and CMake 3.16 or newer. The checked-in CMake configuration selects SDK version `2.1.1`, toolchain version `14_2_Rel1`, and picotool version `2.1.1` for the Pico VS Code workflow.

## Repository layout

```text
CMakeLists.txt             Top-level Pico SDK build configuration
pico_sdk_import.cmake      Pico SDK import helper

includes/
  RobotConfig.h            Explorer geometry, gait settings, joint-to-servo mapping, loop timing
  GaitController.h         Wrapper around gait_core::GaitEngine
  Servos.h                 Servo 2040 servos, contact switches, current sensing
  Imu.h                    IMU reading and Madgwick orientation filter
  Ism330dhcx.h             ISM330DHCX driver (port of hexapod-ros2's)
  ConfigManager.h          Flash-backed joint calibration interface
  DataPacket.h             Packed packets and I²C declarations
  Hexapod.h                Main application include aggregation

src/
  Hexapod.cpp              Application entry point and 100 Hz control loop
  GaitController.cpp       Packets to engine inputs, engine output to servos
  Servos.cpp               Joint angle conversion and Servo 2040 output
  Imu.cpp                  IMU retry/reinitialise and Madgwick filter
  Ism330dhcx.cpp           ISM330DHCX register access
  ConfigManager.cpp        RP2040 flash calibration storage
  DataPacket.cpp           I²C packet exchange and command tracking

external/
  CMakeLists.txt           External dependency entry point
  hexapod-gait-core/       Gait library (submodule, with Eigen as its own submodule)
  pimoroni-pico/           Pimoroni Pico libraries (submodule); only servo2040, button, analogmux and analog are built
```

## How it fits together

`src/Hexapod.cpp` initializes USB stdio, I²C, calibration data and the servos, then `GaitController::begin()` moves each leg in turn into the seated pose. After that, a 100 Hz loop reads a `ControlPacket` from the ESP32, handles newly received commands, reads the IMU, advances the gait, and sends the current `HexPacket` back.

`GaitController` does the job hexapod-ros2's `gait_controller` node does there: it converts the joysticks into a velocity command (or a body pose in tilt mode), passes commands to the engine, calls `GaitEngine::update()` each tick, and hands the 18 joint targets to `Servos`. `Servos` converts the engine's joint angles (radians) into servo angles (degrees), adds the calibration offsets and writes them to the Servo 2040.

```text
ESP32 controller (0x08)        ISM330DHCX (0x6A)
      │                              │
      └──────── I²C (i2c0) ──────────┘
      │                              │
DataPacket.cpp                     Imu ── MadgwickFilter
      │                              │ roll, pitch, yaw
      ▼                              ▼
Hexapod.cpp (100 Hz loop) ──► GaitController ──► gait_core::GaitEngine (hexapod-gait-core)
                                     │
                                     ▼
                     Servos ── ConfigManager (offsets) ── Servo 2040 cluster
```

| hexapod-ros2 | Here |
| --- | --- |
| `ism330dhcx_node` | `Ism330dhcx` + `Imu` (same registers, ranges, retry and reinitialise) |
| `imu_filter_madgwick` (no magnetometer, ENU, gain 0.1) | `MadgwickFilter` |
| `gait_controller` node | `GaitController` |
| `/cmd_vel`, `/body_pose` from `controller_teleop` | Joysticks in `ControlPacket` |
| `st3215` hardware interface | `Servos` |

The servos are hobby PWM servos with no position or effort feedback. The engine is told once at boot that the legs are in the seated pose (where `begin()` has just put them) and runs open loop after that. Without effort there are no foot forces, so load balancing and terrain adaptation are switched off in `RobotConfig.h`, and the engine's leg odometry has nothing useful to work from.

### IMU

The ISM330DHCX plugs into the Servo 2040's Qw/ST connector, which is the same I²C bus (GPIO 20/21) as the ESP32. It must be mounted with its axes along the body's (x forward, y left, z up), like `imu_link` in the URDF. It is read every tick and filtered into roll, pitch and yaw, which `GaitController::onImu()` passes to the engine. Yaw drifts without a magnetometer; the engine only uses it when `tilt_lock_yaw` is on (off by default).

If the IMU is missing, the firmware tries again once a second and prints why over USB serial. The engine ignores IMU data more than 0.2 s old, so the robot walks unleveled until the IMU answers.

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
CMD_HOME_STANCE      stand up
CMD_REQUEST_CONFIG   report leg commandArgs[0]'s calibration offsets in HexPacket.legConfigs
CMD_SIT              commandArgs[0]: 1 sit down, 0 stand up (new, appended so existing values are unchanged)
```

`Gait` and `Mode` values are the same as gait-core's, so they pass straight to the engine. The firmware handles `MODE_CONFIG` itself.

`ControlPacket.currentHeight` (mm, 0 = not set) sets the body height when it changes. The engine accepts 60-160 mm, and only while standing. `HexPacket.currentHeight` reports the body height in mm. `HexPacket.currentPhase` is always 0, because `GaitEngine` does not expose its phase.

`CommandChanged()` compares the current command and its three arguments with the previous values, preventing the same command from being repeatedly applied while the packet remains unchanged.

## Motion control

The gait is hexapod-gait-core's `GaitEngine`; see that repository for how it works. Its settings (`GaitParams`) are set in `robot::gaitParams()`; anything not set there keeps the hexapod-ros2 default.

### Joysticks

Joystick values run to ±128 with a dead zone of 10. X positive is right and Y positive is forward. Full stick is the gait's top speed (`GaitEngine::maxFootSpeed()`), the same scale hexapod-ros2's teleop uses.

| Mode | Left stick | Right stick X | Right stick Y |
| --- | --- | --- | --- |
| `MODE_NORMAL` | Y: forward/back, X: turn | turn | — |
| `MODE_STRAFE` | forward/back and sideways | turn | — |
| `MODE_TILT` | shift the body | roll | pitch |
| `MODE_CONFIG` | — | — | — |

The velocity is smoothed like hexapod-ros2's `cmd_vel`, and a centred stick stops it at once. If 20 I²C reads in a row fail (200 ms), the sticks count as centred.

### Gaits

| Gait | Legs that swing together |
| --- | --- |
| Tripod | `{1, 3, 5}` and `{2, 4, 6}` |
| Ripple | `{3, 6}`, `{2, 4}`, and `{1, 5}` |
| Wave | `{3}`, `{2}`, `{1}`, `{4}`, `{5}`, and `{6}` |

Gait changes apply at the next step.

### Startup, sitting and standing

The engine starts seated. `stand_on_startup` is on, so it stands up as soon as startup finishes (hexapod-ros2 waits to be told). `CMD_SIT` sits it down or stands it up; `CMD_HOME_STANCE` stands it up. The engine refuses height changes and tilt mode while seated.

### Calibration pose

`MODE_CONFIG` moves every servo to 0° (plus its calibration offset) at 90°/s, for setting offsets with `CMD_SET_CONFIG`. Leaving it slews the servos back to the engine's output at the same speed.

## Servo 2040 output

The leg-to-servo mapping uses the gait-core leg order (clockwise from the front right, `leg1`-`leg6` in hexapod-ros2):

```text
Leg 1 (front right):  servos 0, 1, 2     (coxa, femur, tibia)
Leg 2 (middle right): servos 3, 4, 5
Leg 3 (rear right):   servos 6, 7, 8
Leg 4 (rear left):    servos 9, 10, 11
Leg 5 (middle left):  servos 12, 13, 14
Leg 6 (front left):   servos 15, 16, 17
```

Joint angles are converted as

```text
servo (deg) = JOINT_DIRECTION × (joint (deg) − JOINT_CENTRE_DEG) + calibration offset
```

clamped to ±90°. The directions are hexapod-ros2's (`tibia_direction = -1`). The tibia servo is centred with the knee at 90°, because a ±90° PWM servo can't otherwise reach the tibia's working range (the Explorer stands at about 110°). The engine's femur and tibia limits are narrowed to what the servos can reach. If a joint turns the wrong way on the robot, flip its sign in `robot::JOINT_DIRECTION`.

## Ground-contact switches and current sensing

Each leg is assigned a Servo 2040 analog-multiplexer address (`Servos::switchPressed()`, `Servos::allLegsGrounded()`):

```text
Leg 1: SENSOR_1_ADDR
Leg 2: SENSOR_2_ADDR
Leg 3: SENSOR_3_ADDR
Leg 4: SENSOR_4_ADDR
Leg 5: SENSOR_5_ADDR
Leg 6: SENSOR_6_ADDR
```

`Servos::init()` configures pull-down resistors for the leg switches. The gait does not use them yet.

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

The firmware loads offsets during startup, and `Servos::write()` adds them to every servo output. `CMD_SET_CONFIG` arguments are checked: leg 0-5, joint 0-2.

> Confirm the configured flash offset in `includes/ConfigManager.h` does not overlap the application image or other persistent data before changing the build layout.

## Building

### Prerequisites

Install:

- Git
- CMake 3.16 or newer
- Ninja or Make
- ARM GCC toolchain for RP2040
- Raspberry Pi Pico SDK 2.0.0 or newer
- Picotool
- A USB cable for the Servo 2040

The project can also be opened with the Raspberry Pi Pico VS Code extension. The top-level `CMakeLists.txt` expects the Pico SDK to be discoverable through the normal SDK environment, CMake cache, or the generated VS Code integration.

### Get the source

hexapod-gait-core (with Eigen as its own submodule) and pimoroni-pico are git submodules:

```bash
git clone --recursive <this repository>
# or, in an existing clone
git submodule update --init --recursive
```

`--recursive` also fetches pimoroni-pico's own submodules (sensor drivers and MicroPython modules). The firmware doesn't need them, so `git submodule update --init` followed by `git -C external/hexapod-gait-core submodule update --init` is enough.

A build directory configured before the move to SDK 2 has the old SDK in its CMake cache: delete it and configure again.

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

At startup, `main()` waits five seconds, then moves each leg straight to the seated pose, one leg every 250 ms. The engine then glides to its startup pose and stands up. Before the first power-on, check that `sit_reach` and `sit_height` (gait-core defaults: 220 mm out, 15 mm below the hips) suit the robot.

## Troubleshooting

### CMake cannot find the Pico SDK

- Set `PICO_SDK_PATH` to a valid Pico SDK checkout.
- Confirm that the SDK is version 2.0.0 or newer. On SDK 1.x the Pimoroni code fails with errors such as `'pio_set_gpio_base' was not declared`.
- Remove and recreate the build directory after changing the SDK path.
- If using VS Code, verify the Pico extension's SDK and toolchain settings.

### The build cannot find Servo 2040 libraries

- Confirm that `external/pimoroni-pico` is checked out: `git submodule update --init`.
- Check that `external/CMakeLists.txt` includes the Pimoroni library `.cmake` files. Don't `add_subdirectory()` the whole repository: its CMakeLists.txt builds every example, which needs its nested submodules.
- Verify that the linked targets include `servo2040`, `analogmux`, `analog`, `button`, and `gait_core`.

### CMake reports "Eigen not found"

- Run `git submodule update --init --recursive`; Eigen is a submodule of hexapod-gait-core.

### I²C packets are not exchanged

- Confirm that the ESP32 is configured as address `0x08`.
- Check SDA/SCL wiring and shared ground.
- Verify GPIO 20 and GPIO 21 are available on the hardware configuration.
- Confirm that both sides use identical packed `ControlPacket` and `HexPacket` layouts.
- Inspect USB serial output for `ReadInputData()` byte-count errors.

### The IMU is not found or leveling goes the wrong way

- USB serial prints `[imu] ISM330DHCX not found at 0x6A (WHO_AM_I 0x..)` once a second while it's missing. `0x00` means nothing answered: check the Qw/ST cable, or set `IMU_ADDRESS` to `0x6B` if the address jumper is bridged.
- If the body tilts further instead of levelling, the IMU's axes don't match the body's (x forward, y left, z up). Mount it that way, or the leveling will fight the slope.

### Servos do not move correctly

- Check the leg-to-servo mapping in `src/Servos.cpp` and the joint directions in `includes/RobotConfig.h`.
- Verify mechanical servo orientation and mirrored-leg settings.
- Confirm that calibration offsets were loaded correctly.
- Check the Servo 2040 supply voltage and current capacity.
- Test `MODE_CONFIG` (every servo at 0°) with the robot supported safely.

### Calibration values are invalid after flashing

- Confirm the flash sector reserved by `FLASH_TARGET_OFFSET` is not part of the application image.
- Erasing or reprogramming the reserved sector resets stored offsets.
- Reapply calibration values using `CMD_SET_CONFIG` after a deliberate reset.

## Development notes

- The top-level executable is `Hexapod`, defined in `src/Hexapod.cpp`.
- USB stdio is enabled; UART stdio is disabled. Engine log messages are printed there as `[gait INFO] ...`.
- The loop runs at 100 Hz (`robot::LOOP_PERIOD_US`). An overrun tick skips ahead rather than catching up.
- Gait changes belong in hexapod-gait-core so hexapod-ros2 gets them too. To update it, check out a newer tag in `external/hexapod-gait-core` and commit the submodule.
- pimoroni-pico is a submodule tracking Pimoroni's `main`. To update it, check out a newer commit in `external/pimoroni-pico`, rebuild, and commit the submodule.
- If the servo cluster fails to start, the firmware stops after the boot wait and prints why over USB serial every 2 s.
- No top-level license file is currently present.

## Contributing

1. Create a feature branch.
2. Keep packet layouts compatible with the ESP32 firmware.
3. Put gait changes in hexapod-gait-core rather than here.
4. Build with CMake before submitting changes.
5. Test with the robot mechanically supported.
6. Document any changes to pin mappings, servo order, I²C settings, gait behavior, or flash layout.

## License

No top-level license file is currently included. Unless a license is added, the project should be treated as all rights reserved. pimoroni-pico is MIT-licensed, and its individual components may have separate licenses; consult the relevant files under `external/pimoroni-pico` before redistributing them. hexapod-gait-core is Apache-2.0.
