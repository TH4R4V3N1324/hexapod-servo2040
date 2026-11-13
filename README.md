# hexapod-servo2040

Control firmware for the Explorer hexapod on a Pimoroni Servo 2040 (RP2040). It runs the gait
from [hexapod-gait-core](https://github.com/TH4R4V3N1324/hexapod-gait-core), drives 18 hobby PWM
servos, and takes commands from an ESP32 over I²C.

The robot and its build are documented in
[hexapod-docs](https://github.com/TH4R4V3N1324/hexapod-docs). This README only covers building
and working on the firmware.

## Quick start

You need the Raspberry Pi Pico SDK **2.0.0 or newer** (the project selects SDK `2.1.1`,
toolchain `14_2_Rel1` and picotool `2.1.1`), CMake 3.16+, and Ninja or Make. The Raspberry Pi
Pico VS Code extension sets all of these up from `CMakeLists.txt`.

```sh
git clone https://github.com/TH4R4V3N1324/hexapod-servo2040.git
cd hexapod-servo2040
git submodule update --init
git -C external/hexapod-gait-core submodule update --init   # Eigen

cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build
```

(`git clone --recursive` works too, but also fetches pimoroni-pico's own submodules, which the
firmware doesn't use.)

Hold BOOTSEL on the Servo 2040 while plugging it in, and copy `build/Hexapod.uf2` to the
`RPI-RP2` drive (or use `picotool load -x build/Hexapod.uf2`). Diagnostics are printed over USB
serial.

**Before powering the servos,** read [Safety](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/getting-started/safety.md)
and support the robot so its legs are clear of the ground. Five seconds after boot, the firmware moves
each leg in turn into the seated pose, then stands up.

## Repository layout

```text
includes/, src/
  Hexapod            Entry point and 100 Hz control loop
  RobotConfig.h      Explorer geometry, GaitParams, joint-to-servo mapping, timings, IMU settings
  GaitController     ESP32 packets → GaitEngine inputs; engine output → servos
  Servos             Joint angles ↔ servo angles, Servo 2040 output, switches, current sense
  Imu, Ism330dhcx    ISM330DHCX driver and Madgwick filter
  DataPacket         I²C packets exchanged with the ESP32
  ConfigManager      Calibration offsets in RP2040 flash
external/
  hexapod-gait-core  Submodule, pinned to a release tag (v0.5.0)
  pimoroni-pico      Submodule, tracking Pimoroni's main
```

## How it works

Each 100 Hz tick, `Hexapod.cpp` reads a `ControlPacket` from the ESP32, reads the IMU, and calls
`GaitController::update()`, which runs `GaitEngine::update()` and writes the 18 joint targets
through `Servos`. It then sends a `HexPacket` back. An overrun tick skips ahead rather than
catching up.

The servos have no position or effort feedback. The engine is told once, at boot, that the legs
are in the seated pose `begin()` has just put them in, and runs open loop after that. With no foot
forces, `load_balancing` and `terrain_adaptation` are off in `robot::gaitParams()`. Settings not
set there keep gait-core's defaults.

### Servo power and battery

There is no servo power switch, so the engine's `servoPower` output starts and stops the PWM
signals: off, the servos go limp. Powering back up (any stand command) puts the legs in the
seated pose, as at boot, before the engine stands. The engine's lost-servo check
(`servo_loss_timeout`) is off, because it works by joint states stopping and these servos never
send any.

`robot::BATTERY_MONITORING` feeds the Servo 2040's supply voltage and current to gait-core's
`BatteryMonitor`, which warns when it's low and sits down and stops the servos at critical. It's
off until `robot::batteryParams()` has thresholds for
whatever feeds the servo rail: the defaults are for a 3S pack.

### Joint-to-servo mapping

Leg *n* (gait-core order: 0 = front right, clockwise) uses servo channels 3*n* to 3*n*+2 (coxa,
femur, tibia).

```text
servo° = JOINT_DIRECTION × (joint° − JOINT_CENTRE_DEG) + calibration offset, clamped to ±90°
```

`JOINT_DIRECTION` is `{1, 1, −1}` (the tibia servo turns the other way). The tibia is
centred at 90° because a ±90° servo can't otherwise reach its working range, and the femur and
tibia limits in `robot::geometry()` are narrowed to what the servos can reach. Both are set in
`RobotConfig.h`.

### IMU

The ISM330DHCX (`0x6A`) shares `i2c0` (GPIO 20/21) with the ESP32 and must be mounted with its axes along
the body's (x forward, y left, z up). If it's missing, the firmware retries once a second and walks
without levelling.

## ESP32 interface

The packets in `includes/DataPacket.h` are packed and must stay byte-compatible with the ESP32
firmware. The Servo 2040 is the I²C master at 100 kHz; the ESP32 is at `0x08`.

```cpp
struct ControlPacket {           // ESP32 → Servo 2040
    int16_t joystick1X, joystick1Y, joystick2X, joystick2Y;  // ±128, X right, Y forward
    int16_t currentHeight;       // body height in mm, 0 = unchanged
    Command command;
    int16_t commandArgs[3];
};

struct HexPacket {               // Servo 2040 → ESP32
    int16_t legConfigs[3];       // calibration offsets requested with CMD_REQUEST_CONFIG
    int16_t currentHeight;       // mm
    int8_t currentPhase;         // always 0
    Gait currentGait;
    Mode currentMode;
};
```

| Command | Arguments |
| --- | --- |
| `CMD_SET_GAIT` | `[0]` gait |
| `CMD_SET_MODE` | `[0]` mode. `MODE_CONFIG` (all servos to 0° + offset) is handled here, not by the engine |
| `CMD_SET_CONFIG` | `[0]` leg 0–5, `[1]` joint 0–2, `[2]` offset in degrees (clamped to ±60) |
| `CMD_HOME_STANCE` | Stand up |
| `CMD_REQUEST_CONFIG` | `[0]` leg; its offsets are returned in `legConfigs` |
| `CMD_SIT` | `[0]` 1 sit, 0 stand |
| `CMD_POWER_DOWN` | Sit, then stop the servos. A stand (`CMD_SIT 0` or `CMD_HOME_STANCE`) starts them again |

A command is applied once, when it or its arguments change. `Gait` and `Mode` values equal
gait-core's (checked with `static_assert`). If 20 reads in a row fail (200 ms), the sticks count
as centred. New commands must be appended so existing values don't change.

## Development notes

- **Gait changes belong in hexapod-gait-core**, not here. To update it, check
  out a newer tag in `external/hexapod-gait-core`, rebuild, and commit the submodule.
- **pimoroni-pico:** `external/CMakeLists.txt` includes only the `servo2040`, `button`, `analogmux`
  and `analog` `.cmake` files. Don't `add_subdirectory()` the whole repository: it builds every
  example, which needs its nested submodules. To update it, check out a newer commit, rebuild, and
  commit the submodule.
- **Calibration** lives in a 4 KiB flash sector at `FLASH_TARGET_OFFSET` (`ConfigManager.h`). Make
  sure it stays clear of the application image if the build layout changes.
- **Leg switches** on the sensor inputs (`Servos::switchPressed()`) are read but not used by the
  gait yet.
- If the servo cluster fails to start, the firmware stops after the boot wait and prints the reason
  over USB serial every 2 s.

### Build troubleshooting

- `'pio_set_gpio_base' was not declared` or similar: the SDK is 1.x. Use 2.0.0 or newer, and
  delete the build directory, since CMake caches the SDK path.
- `Eigen not found`: run `git -C external/hexapod-gait-core submodule update --init`.

## License

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE). The submodules have their own licences:
hexapod-gait-core is Apache-2.0 and pimoroni-pico is MIT.
