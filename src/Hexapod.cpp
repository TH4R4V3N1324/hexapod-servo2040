#include "Hexapod.h"

#include <algorithm>

Servos servos;
GaitController controller(servos);
Imu imu(I2C_PORT, robot::IMU_ADDRESS);  // shares the ESP32's I2C bus

int main() {
    stdio_init_all();
    InitI2C();
    configManager.loadLegOffsets();
    const bool servosReady = servos.init();

    sleep_ms(5000);
    if (!servosReady) {
        // Nothing can move: say why over USB serial rather than run the gait blind
        while (1) {
            printf("[servos] Servo cluster failed to initialise (out of memory, or its pins are in use)\n");
            sleep_ms(2000);
        }
    }
    controller.begin();

    uint64_t lastTick = time_us_64();
    absolute_time_t nextTick = make_timeout_time_us(robot::LOOP_PERIOD_US);
    int readFailures = 0;

    while (1) {
        const bool received = ReadInputData();
        readFailures = received ? 0 : readFailures + 1;
        const bool commsOk = readFailures < robot::COMMS_TIMEOUT_TICKS;
        if (received && CommandChanged()) {
            controller.handleCommand(controlPacket);
        }

        const uint64_t now = time_us_64();
        const gait_core::Scalar dt = std::clamp(static_cast<gait_core::Scalar>(now - lastTick) / 1e6f,
                                                gait_core::Scalar(0), robot::MAX_TICK_DT);
        lastTick = now;

        if (imu.update(now)) {
            float roll, pitch, yaw;
            imu.rollPitchYaw(roll, pitch, yaw);
            controller.onImu(roll, pitch, yaw, now);
        }
        controller.update(dt, now, controlPacket, commsOk);

        controller.fillTelemetry(hexPacket);
        SendHexData();

        sleep_until(nextTick);
        nextTick = delayed_by_us(nextTick, robot::LOOP_PERIOD_US);
        // An overrun tick skips ahead rather than running the missed ones back to back
        if (absolute_time_diff_us(get_absolute_time(), nextTick) < 0) {
            nextTick = make_timeout_time_us(robot::LOOP_PERIOD_US);
        }
    }
}
