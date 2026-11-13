#ifndef _IMU_H_
#define _IMU_H_

#include <cstdint>

#include "hardware/i2c.h"

#include "Ism330dhcx.h"

/**
 * @brief Madgwick orientation filter, accelerometer and gyroscope only
 * @note The orientation is the sensor's relative to a level frame with z up; yaw is relative to
 *       the heading at the first sample and drifts, having no magnetometer.
*/
class MadgwickFilter {
public:
    explicit MadgwickFilter(float gain) : gain(gain) {}

    /**
     * @brief Add a reading; the first one sets the roll and pitch straight from the accelerometer
     * @param[in] gyro Angular velocity (rad/s)
     * @param[in] accel Acceleration (m/s², any scale), ignored if zero
     * @param[in] dt Time since the last reading (s)
    */
    void update(const float gyro[3], const float accel[3], float dt);
    void reset() { initialised = false; }

    /**
     * @brief Roll, pitch and yaw (rad) from the quaternion
    */
    void rollPitchYaw(float& roll, float& pitch, float& yaw) const;

private:
    float gain;
    bool initialised = false;
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // w, x, y, z: sensor to level frame
};

/**
 * @brief The IMU: the ISM330DHCX and its orientation filter
 * @note Retries a missing sensor once a second and re-initialises after a failed read.
*/
class Imu {
public:
    Imu(i2c_inst_t* bus, uint8_t address);

    /**
     * @brief Read the sensor if it's time, and update the orientation
     * @param[in] nowUs The current time (us since boot)
     * @return true if there is a new orientation
    */
    bool update(uint64_t nowUs);

    void rollPitchYaw(float& roll, float& pitch, float& yaw) const { filter.rollPitchYaw(roll, pitch, yaw); }

private:
    i2c_inst_t* bus;
    uint8_t address;
    Ism330dhcx sensor;
    MadgwickFilter filter;

    bool ready = false;
    uint64_t nextInitAttemptUs = 0;
    uint64_t lastReadUs = 0;
    bool hasLastRead = false;
};

#endif
