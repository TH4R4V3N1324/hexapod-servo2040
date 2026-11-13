#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include <algorithm>
#include <array>
#include <cstdint>

#include "gait_core/gait_engine.hpp"
#include "gait_core/robot_geometry.hpp"
#include "hexapod_behaviour/battery_monitor.hpp"

/**
 * Everything specific to running the Explorer on a Servo 2040: its shape (gait_core's preset),
 * the gait settings and how gait_core's joint angles map onto the PWM servos.
 *
 * gait_core works in SI units with the body frame +x forward, +y left, +z up, and legs indexed
 * 0-5 clockwise from the front right (leg1-leg6 on the robot):
 *
 *   gait_core::Leg       leg        servos
 *   LEG_FRONT_RIGHT  0   leg1       0, 1, 2      (coxa, femur, tibia)
 *   LEG_MIDDLE_RIGHT 1   leg2       3, 4, 5
 *   LEG_REAR_RIGHT   2   leg3       6, 7, 8
 *   LEG_REAR_LEFT    3   leg4       9, 10, 11
 *   LEG_MIDDLE_LEFT  4   leg5       12, 13, 14
 *   LEG_FRONT_LEFT   5   leg6       15, 16, 17
*/
namespace robot {

using gait_core::Scalar;

// ---------------------------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------------------------

/**
 * @brief The Explorer (gait_core's preset), with the joint limits narrowed to what a
 *        +-90 degree PWM servo can reach given the joint mapping below
*/
inline gait_core::RobotGeometry geometry() {
    gait_core::RobotGeometry g = gait_core::RobotGeometry::explorer();
    // femur: servo = femur, so +-90 deg (preset: -1.6 to 1.4)
    g.limits.femurMin = std::max(g.limits.femurMin, Scalar(-1.52));
    g.limits.femurMax = std::min(g.limits.femurMax, Scalar(1.52));
    // tibia: servo = -(tibia - 90 deg), so 0 to 180 deg (preset: -0.78 to 2.63)
    g.limits.tibiaMin = std::max(g.limits.tibiaMin, Scalar(0.05));
    g.limits.tibiaMax = std::min(g.limits.tibiaMax, Scalar(3.09));
    return g;
}

// ---------------------------------------------------------------------------------------------
// Gait settings
// ---------------------------------------------------------------------------------------------

/**
 * @brief GaitEngine settings; anything not set here keeps gait_core's default
*/
inline gait_core::GaitParams gaitParams() {
    gait_core::GaitParams p;

    // The ESP32 controller has no sit/stand button yet, so stand up as soon as startup has
    // seated the robot
    p.stand_on_startup = true;

    // Leveling (walk_leveling, tilt_leveling) uses the IMU. Hobby servos report
    // no effort, so there are no foot forces: load balancing and terrain adaptation have nothing to
    // work from. The engine would skip them anyway; turning them off saves the RP2040 (no FPU) the work.
    p.load_balancing = false;
    p.terrain_adaptation = false;

    // Servo power: there's no power switch, so "off" is the PWM signals stopped (the servos go limp).
    // They're on from boot, as before. The engine notices lost servos (e-stop) by joint states stopping,
    // but these servos never report any, so that check is off: it would cut power 0.5 s after boot.
    p.power_on_startup = true;
    p.servo_loss_timeout = 0;
    return p;
}

// ---------------------------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------------------------

// Watch the Servo 2040's servo supply voltage with gait_core's BatteryMonitor: warn when it's low, and
// at critical sit down and stop the servos. Off until the thresholds below suit what feeds the servo
// rail: gait_core's defaults are for a 3S pack and and would sit a 6 V rail down at once.
constexpr bool BATTERY_MONITORING = false;

inline hexapod_behaviour::BatteryParams batteryParams() {
    hexapod_behaviour::BatteryParams p;
    // p.warning_voltage = ...;   // V
    // p.critical_voltage = ...;  // V
    return p;
}

// ---------------------------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------------------------

// ISM330DHCX on the Servo 2040's Qw/ST connector, which is the ESP32's I2C bus (i2c0, GPIO 20/21).
// Mounted with its axes along the body's (x forward, y left, z up).
constexpr uint8_t IMU_ADDRESS = 0x6A;        // 0x6B if the address jumper is bridged
constexpr int IMU_ACCEL_RANGE_G = 8;         // 2, 4, 8 or 16
constexpr int IMU_GYRO_RANGE_DPS = 1000;     // 125, 250, 500, 1000, 2000 or 4000
constexpr float IMU_FILTER_GAIN = 0.1f;      // Madgwick filter gain

// Gait loop rate. The I2C exchange with the ESP32 takes ~3 ms of each tick.
constexpr uint32_t LOOP_PERIOD_US = 10000;  // 100 Hz
constexpr Scalar MAX_TICK_DT = Scalar(0.1); // s, a stalled tick can't make the legs jump further than this

// ---------------------------------------------------------------------------------------------
// Controller input
// ---------------------------------------------------------------------------------------------

constexpr int JOYSTICK_FULL_SCALE = 128;   // joystick value at full deflection
constexpr int JOYSTICK_DEADZONE = 10;      // values this close to centre count as centred
constexpr Scalar VELOCITY_SMOOTHING = Scalar(0.5);  // per tick, low-pass filter on the stick velocity
constexpr int COMMS_TIMEOUT_TICKS = 20;    // failed I2C reads in a row before the sticks are centred

// ---------------------------------------------------------------------------------------------
// Servos
// ---------------------------------------------------------------------------------------------

/**
 * @brief servo (deg) = JOINT_DIRECTION * (joint (deg) - JOINT_CENTRE_DEG), then the calibration offset
 * @note A +-90 degree PWM servo centred with the tibia in line with the femur can't reach the
 *       tibia's working range (it stands at ~110 degrees), so the tibia servo is centred with the
 *       knee at 90 degrees. If a joint turns the wrong way on the robot, flip its direction.
*/
constexpr std::array<float, 3> JOINT_DIRECTION = {1, 1, -1};      // coxa, femur, tibia
constexpr std::array<float, 3> JOINT_CENTRE_DEG = {0, 0, 90};     // coxa, femur, tibia

// Servo angle limit (deg); the Servo 2040 default angular calibration spans +-90
constexpr float SERVO_LIMIT_DEG = 90.0f;

// Speed (deg/s) servos move at into and out of the calibration pose (MODE_CONFIG: every servo at 0)
constexpr float CONFIG_SLEW_DEG_PER_S = 90.0f;

// Pause between powering each leg at boot, so all 18 servos don't start at once. Each leg goes
// straight to the seated pose (sit_reach, sit_height), which the engine then stands up from.
constexpr uint32_t BOOT_LEG_STAGGER_MS = 250;

}  // namespace robot

#endif
