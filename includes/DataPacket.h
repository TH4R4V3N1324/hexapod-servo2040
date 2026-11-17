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

// ControlPacket, HexPacket, Command, Gait, Mode, GaitFeature and HEX_FLAG_*: the packets every
// controller and robot share, from hexapod-gait-core (include/gait_core/protocol/packets.h)
#include <gait_core/protocol/packets.h>

extern ControlPacket controlPacket;
extern HexPacket hexPacket;

bool ReadInputData();
void SendHexData();
void InitI2C();
bool CommandChanged();

#endif