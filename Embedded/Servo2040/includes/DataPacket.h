#ifndef _DATAPACKET_H_
#define _DATAPACKET_H_

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

// I2C address and port of the ESP32
#define ESP32_SLAVE_ADDR 0x08
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21

enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

enum Mode : uint8_t {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    MODE_CONFIG,
    NUM_MODES
};

enum Command : uint8_t {
    CMD_NONE = 0,
    CMD_SET_GAIT,
    CMD_SET_MODE,
    CMD_SET_CONFIG,
    CMD_HOME_STANCE,
	CMD_REQUEST_CONFIG
};

// Define the data structure with no padding
#pragma pack(push, 1)
// Define ControlPacket struct
struct ControlPacket {
    int16_t joystick1X;
    int16_t joystick1Y;
    int16_t joystick2X;
    int16_t joystick2Y;
    int16_t currentHeight;
    Command command;
    int16_t commandArgs[3];
};

// Define HexPacket struct
struct HexPacket {
    int16_t legConfigs[3];
    int16_t currentHeight;
    int8_t currentPhase;
    Gait currentGait;
    Mode currentMode;
};
#pragma pack(pop)

extern ControlPacket controlPacket;
extern HexPacket hexPacket;

void ReadInputData();
void SendHexData();
void InitI2C();
bool CommandChanged();

#endif