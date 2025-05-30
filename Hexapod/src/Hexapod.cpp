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
    i2c_write_blocking(I2C_PORT, ESP32_SLAVE_ADDR, NULL, 0, true);

    int bytesRead = i2c_read_blocking(I2C_PORT, ESP32_SLAVE_ADDR, (uint8_t*)&receivedData, sizeof(receivedData), false);

    if (bytesRead != sizeof(receivedData)) {
        printf("Failed to read data from ESP32. Bytes read: %d\n", bytesRead);
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

    Animation animation;
    Move move;
    Calculate cal;

    sleep_ms(5000);
    animation.Startup();

    while (1) {
        ReadInputData();
        // Safely read shared data
        int lx = receivedData.LStickX;
        int ly = receivedData.LStickY;
        //receivedData.LStickX = 0; // Reset to avoid repeated processing
        //receivedData.LStickY = 128; // Reset to avoid repeated processing

        printf("LStickX: %d, LStickY: %d\n", lx, ly);
        // ...robot logic...
        animation.Strafe();
        
        sleep_ms(1);
    }
}