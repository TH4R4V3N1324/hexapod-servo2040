#include "DataPacket.h"

// Definitions for data packets
ControlPacket controlPacket;
HexPacket hexPacket;

// Requests data from ESP32 and writes data to controlData structure
void ReadInputData() {
    i2c_write_blocking(I2C_PORT, ESP32_SLAVE_ADDR, NULL, 0, true);

    int bytesRead = i2c_read_blocking(I2C_PORT, ESP32_SLAVE_ADDR, (uint8_t*)&controlPacket, sizeof(controlPacket), false);

    if (bytesRead != sizeof(controlPacket)) {
        printf("Failed to read data from ESP32. Bytes read: %d\n", bytesRead);
    }
}

// Sends hexapod data to ESP32 stored in hexPacket
void SendHexData() {
    i2c_write_blocking(I2C_PORT, ESP32_SLAVE_ADDR, (uint8_t*)&hexPacket, sizeof(hexPacket), false);
}

// Initialize I2C as master
void InitI2C() {
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Returns true if the a ControlPacket command changes
bool CommandChanged() {
    bool changed = (controlPacket.command != lastCommand) ||
                   (controlPacket.commandArgs[0] != lastArgs[0]) ||
                   (controlPacket.commandArgs[1] != lastArgs[1]) ||
                   (controlPacket.commandArgs[2] != lastArgs[2]);

    if (changed) {
        lastCommand = controlPacket.command;
        lastArgs[0] = controlPacket.commandArgs[0];
        lastArgs[1] = controlPacket.commandArgs[1];
        lastArgs[2] = controlPacket.commandArgs[2];
    }
    return changed;
}