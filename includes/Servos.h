#ifndef _SERVOS_H_
#define _SERVOS_H_

#include <array>

#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"

#include "gait_core/kinematic_solver.hpp"
#include "gait_core/robot_geometry.hpp"

/**
 * @brief Servo angles of one leg in degrees: coxa, femur, tibia
*/
using ServoDegrees = std::array<float, 3>;

/**
 * @brief The Servo 2040 hardware: the 18 leg servos, the leg contact switches and current sensing
 * @note Legs are indexed like gait_core (0 = front right, clockwise). Leg n uses servos 3n to 3n+2.
*/
class Servos {
private:
    static constexpr uint START_PIN = servo::servo2040::SERVO_1;
    static constexpr uint END_PIN = servo::servo2040::SERVO_18;
    static constexpr uint NUM_SERVOS = (END_PIN - START_PIN) + 1;

    servo::ServoCluster cluster = servo::ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    AnalogMux mux = AnalogMux(servo::servo2040::ADC_ADDR_0, servo::servo2040::ADC_ADDR_1, servo::servo2040::ADC_ADDR_2,
                              PIN_UNUSED, servo::servo2040::SHARED_ADC);
    Analog cur_adc = Analog(servo::servo2040::SHARED_ADC, servo::servo2040::CURRENT_GAIN,
                            servo::servo2040::SHUNT_RESISTOR, servo::servo2040::CURRENT_OFFSET);

    // Contact switch of each leg, on the analog mux sensor inputs
    static constexpr std::array<uint, gait_core::NUM_LEGS> legSwitch = {
        servo::servo2040::SENSOR_1_ADDR, servo::servo2040::SENSOR_2_ADDR, servo::servo2040::SENSOR_3_ADDR,
        servo::servo2040::SENSOR_4_ADDR, servo::servo2040::SENSOR_5_ADDR, servo::servo2040::SENSOR_6_ADDR,
    };

public:
    /**
     * @return false if the servo cluster couldn't start (out of memory, or its pins are in use)
    */
    bool init();

    /**
     * @brief gait_core joint angles (rad) to servo angles (deg), before calibration offsets
    */
    static ServoDegrees toServoDegrees(int legNum, const gait_core::JointAngles& angles);
    /**
     * @brief Servo angles (deg, before calibration offsets) back to gait_core joint angles (rad)
    */
    static gait_core::JointAngles toJointAngles(int legNum, const ServoDegrees& degrees);

    /**
     * @brief Move a leg's servos, adding its calibration offsets and clamping to the servo range
     * @param[in] legNum The leg (gait_core index)
     * @param[in] degrees Servo angles before calibration offsets
    */
    void write(int legNum, const ServoDegrees& degrees);
    void disable(int legNum);

    bool switchPressed(int legNum);
    bool allLegsGrounded();
    float currentDraw();
};

#endif
