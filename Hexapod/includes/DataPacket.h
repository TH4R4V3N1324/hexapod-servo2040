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

enum Command : uint8_t {
    CMD_NONE = 0,
    CMD_SET_GAIT,
    CMD_SET_MODE,
    CMD_ENTER_CONFIG,
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
    int16_t currentHeight;
    Command command;
    int16_t commandArgs[3];
};

// Define HexPacket struct
struct HexPacket {
    int16_t legConfigs[3];
    int16_t currentHeight;
};
#pragma pack(pop)

extern ControlPacket controlPacket;
extern HexPacket hexPacket;

void ReadInputData();
void SendHexData();
void InitI2C();

#endif