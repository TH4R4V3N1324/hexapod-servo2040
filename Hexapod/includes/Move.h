#ifndef _MOVE_H_
#define _MOVE_H_
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "Calculate.h"
#include "DataPacket.h"
#include <map>
#include <cmath>

struct LegServo {
    int coxa;
    int femur;
    int tibia;
};

class Move{
private:
    const uint START_PIN = servo::servo2040::SERVO_1;
    const uint END_PIN = servo::servo2040::SERVO_18;
    const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
    servo::ServoCluster cluster = servo::ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);
    Analog sen_adc = Analog(servo::servo2040::SHARED_ADC);
    AnalogMux mux = AnalogMux(servo::servo2040::ADC_ADDR_0, servo::servo2040::ADC_ADDR_1, servo::servo2040::ADC_ADDR_2, PIN_UNUSED, servo::servo2040::SHARED_ADC);

    //assigns the relavant servos to their corrosponding leg
    std::map<int, LegServo> legs = {
        {1, {0, 1, 2}},
        {2, {3, 4, 5}},
        {3, {6, 7, 8}},
        {4, {9, 10, 11}},
        {5, {12, 13, 14}},
        {6, {15, 16, 17}}
    };

    //assigns the relavant switch to its corrosponding leg
    std::map<int, const uint> legSwitch = {
        {1, servo::servo2040::SENSOR_1_ADDR},
        {2, servo::servo2040::SENSOR_2_ADDR},
        {3, servo::servo2040::SENSOR_3_ADDR},
        {4, servo::servo2040::SENSOR_4_ADDR},
        {5, servo::servo2040::SENSOR_5_ADDR},
        {6, servo::servo2040::SENSOR_6_ADDR}
    };

public:
    Move();
    Calculate Cal;
    void SetupSwitches();
    bool GetSwitchStatus(int legNum);
    bool AllLegsGrounded();
    Vector3 GetLegPosition(int legNum) const;
    void Position(const Vector3& position, int legNum);
    void StraightLine(const Vector3& end, int legNum);
    void Deactivate(int legNum);
};

#endif 