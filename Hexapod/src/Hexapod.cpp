#include "Hexapod.h"

// I2C address and port of the ESP32
#define ESP32_SLAVE_ADDR 0x08
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21

// Creates button on Servo2040 board
Button user_sw = Button(servo::servo2040::USER_SW);

// Data structure to store received data from ESP32
DataPacket receivedData;

// Requests data from ESP32 and writes data to data structure
void ReadInputData(){
    while (!user_sw.raw()) {
        i2c_write_blocking(I2C_PORT, ESP32_SLAVE_ADDR, NULL, 0, true);
        int bytesRead = i2c_read_blocking(I2C_PORT, ESP32_SLAVE_ADDR, (uint8_t*)&receivedData, sizeof(receivedData), false);

        if (bytesRead != sizeof(receivedData)) {
            printf("Failed to read data from ESP32. Bytes read: %d\n", bytesRead);
        }

        sleep_ms(10);
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
    Animation animation;
    animation.Startup();

    while(!user_sw.raw()){
        sleep_ms(10);
        
        animation.Strafe();
    }

    return 0;
}
