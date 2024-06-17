#include "Hexapod.h"

// I2C address and port of the ESP32
#define ESP32_SLAVE_ADDR 0x08
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

// Creates button on Servo2040 board
Button user_sw = Button(servo::servo2040::USER_SW);

void ReadInputData(){
    // Data structure to store received data from ESP32
    DataPacket receivedData;

    while (!user_sw.raw()) {
        // Request data from ESP32
        i2c_write_blocking(I2C_PORT, ESP32_SLAVE_ADDR, NULL, 0, true);

        // Receive data from ESP32
        int bytesRead = i2c_read_blocking(I2C_PORT, ESP32_SLAVE_ADDR, (uint8_t*)&receivedData, sizeof(receivedData), false);

        if (bytesRead == sizeof(receivedData)) {
            // Print received data
            printf("Bytes read: %d\n", bytesRead);
            printf("Right: %d\n", receivedData.Right);
            printf("Left: %d\n", receivedData.Left);
            printf("Up: %d\n", receivedData.Up);
            printf("Down: %d\n", receivedData.Down);

            printf("Square: %d\n", receivedData.Square);
            printf("Cross: %d\n", receivedData.Cross);
            printf("Circle: %d\n", receivedData.Circle);
            printf("Triangle: %d\n", receivedData.Triangle);

            printf("LStickX: %d\n", receivedData.LStickX);
            printf("LStickY: %d\n", receivedData.LStickY);

            printf("RStickX: %d\n", receivedData.RStickX);
            printf("RStickY: %d\n", receivedData.RStickY);
        } else {
            // Print error if data size does not match expected size
            printf("Failed to read data from ESP32. Bytes read: %d\n", bytesRead);
        }

        sleep_ms(500); // Wait for a second before requesting data again
    }
}

int main() {
    stdio_init_all();

    // Initialize I2C as master
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    multicore_launch_core1(ReadInputData);

    while(!user_sw.raw()){
        sleep_ms(1);
    }

    return 0;
}
