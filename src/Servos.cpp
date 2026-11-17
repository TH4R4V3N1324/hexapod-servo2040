#include "Servos.h"

#include <algorithm>
#include <cmath>

#include "ConfigManager.h"
#include "RobotConfig.h"

namespace {

constexpr float RAD_TO_DEG = 180.0f / gait_core::kPi;
constexpr float DEG_TO_RAD = gait_core::kPi / 180.0f;

}  // namespace

bool Servos::init() {
    for (uint addr : legSwitch) {
        mux.configure_pulls(addr, false, true);  // pull-down
    }
    // Fails if the cluster couldn't allocate its buffers or another PIO state machine has the pins
    return cluster.init();
}

/**
 * @note gait_core's coxa angle is anticlockwise from straight out along the leg,
 *       femur is positive lowering the femur, and tibia is 0 with the tibia in line with the femur.
 *       See robot::JOINT_DIRECTION for how they map onto the servos.
*/
ServoDegrees Servos::toServoDegrees(int /*legNum*/, const gait_core::JointVec& angles) {
    ServoDegrees degrees{};
    const int joints = std::min<int>(angles.size(), robot::JOINTS_PER_LEG);
    for (int joint = 0; joint < joints; ++joint) {
        degrees[joint] = robot::JOINT_DIRECTION[joint] * (angles(joint) * RAD_TO_DEG - robot::JOINT_CENTRE_DEG[joint]);
    }
    return degrees;
}

gait_core::JointVec Servos::toJointAngles(int /*legNum*/, const ServoDegrees& degrees) {
    gait_core::JointVec joints(robot::JOINTS_PER_LEG);
    for (int joint = 0; joint < robot::JOINTS_PER_LEG; ++joint) {
        joints(joint) = (degrees[joint] / robot::JOINT_DIRECTION[joint] + robot::JOINT_CENTRE_DEG[joint]) * DEG_TO_RAD;
    }
    return joints;
}

void Servos::write(int legNum, const ServoDegrees& degrees) {
    // Calibration offsets are stored per leg (0-5) and joint (coxa, femur, tibia), in degrees
    const int16_t* offset = configManager.getLegOffsets(legNum);
    for (int joint = 0; joint < 3; ++joint) {
        const float value = std::clamp(degrees[joint] + offset[joint], -robot::SERVO_LIMIT_DEG, robot::SERVO_LIMIT_DEG);
        cluster.value(legNum * 3 + joint, value);
    }
}

void Servos::disable(int legNum) {
    for (int joint = 0; joint < 3; ++joint) {
        cluster.disable(legNum * 3 + joint);
    }
}

void Servos::disableAll() {
    for (int legNum = 0; legNum < robot::NUM_LEGS; ++legNum) {
        disable(legNum);
    }
}

bool Servos::switchPressed(int legNum) {
    mux.select(legSwitch[legNum]);
    return mux.read();
}

bool Servos::allLegsGrounded() {
    for (int legNum = 0; legNum < robot::NUM_LEGS; ++legNum) {
        if (!switchPressed(legNum)) {
            return false;
        }
    }
    return true;
}

float Servos::currentDraw() {
    mux.select(servo::servo2040::CURRENT_SENSE_ADDR);
    return cur_adc.read_current();
}

float Servos::supplyVoltage() {
    mux.select(servo::servo2040::VOLTAGE_SENSE_ADDR);
    return vol_adc.read_voltage();
}
