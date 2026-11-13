#ifndef _GAIT_CONTROLLER_H_
#define _GAIT_CONTROLLER_H_

#include <array>
#include <cstdint>

#include "gait_core/gait_engine.hpp"

#include "DataPacket.h"
#include "Servos.h"

/**
 * @brief Servo 2040 wrapper around gait_core::GaitEngine, the counterpart of hexapod-ros2's
 *        GaitController node
 * @note The gait itself (startup, walking, sitting, tilt mode, gait changes, idle return) is in
 *       hexapod-gait-core. This class turns the ESP32's packets into engine inputs, ticks the engine,
 *       and writes its joint targets to the servos. It also owns MODE_CONFIG, the calibration pose,
 *       which only this firmware has.
 *
 *       The PWM servos give no position or effort feedback, so the engine is told the legs start in
 *       the seated pose (begin() puts them there) and runs open loop after that. The IMU's
 *       orientation comes in through onImu(), as imuCallback in the ROS node.
*/
class GaitController {
public:
    explicit GaitController(Servos& servos);

    /**
     * @brief Put each leg in the seated pose in turn and tell the engine that's where they are
    */
    void begin();

    /**
     * @brief Act on a command from the ESP32 (call when it changes)
    */
    void handleCommand(const ControlPacket& packet);

    /**
     * @brief Advance the gait one tick and move the servos
     * @param[in] dt Time since the last tick (s)
     * @param[in] nowUs The current time (us since boot)
     * @param[in] packet The latest controller input
     * @param[in] commsOk false once the ESP32 has stopped answering: the sticks count as centred
    */
    void update(gait_core::Scalar dt, uint64_t nowUs, const ControlPacket& packet, bool commsOk);

    /**
     * @brief Pass the body's orientation from the IMU to the engine (rad)
     * @note The engine ignores it once it's 0.2 s old, so leveling stops if the IMU does
    */
    void onImu(float roll, float pitch, float yaw, uint64_t nowUs);

    /**
     * @brief Fill in the telemetry sent back to the ESP32
    */
    void fillTelemetry(HexPacket& packet) const;

private:
    Servos& servos;
    gait_core::GaitEngine engine;

    gait_core::VelocityCommand filteredVelocity{};

    // Servo angles last written to each leg (deg, before calibration offsets)
    std::array<ServoDegrees, gait_core::NUM_LEGS> servoOutput{};

    bool configMode = false;    // MODE_CONFIG: every servo held at 0 for calibration
    bool blending = false;      // slewing between the calibration pose and the engine's output
    int16_t lastHeightRequest = 0;
    int requestedConfigLeg = 0; // leg whose calibration offsets are reported back (CMD_REQUEST_CONFIG)

    void applySticks(const ControlPacket& packet, bool commsOk);
    void applyHeight(int16_t heightMm);
    void writeServos(const gait_core::GaitOutput& output, gait_core::Scalar dt);
};

#endif
