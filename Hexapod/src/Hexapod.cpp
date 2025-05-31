#include "Hexapod.h"

// I2C address and port of the ESP32
#define ESP32_SLAVE_ADDR 0x08
#define I2C_PORT i2c0
#define SDA_PIN 20
#define SCL_PIN 21

// Creates button on Servo2040 board
Button user_sw = Button(servo::servo2040::USER_SW);

// Data structure to store received data from ESP32
DataPacket receivedData = {
    .Right = false,
    .Left = false,
    .Up = false,
    .Down = false,
    .Square = false,
    .Cross = false,
    .Circle = false,
    .Triangle = false,
    .LStickX = 0,
    .LStickY = 0,
    .RStickX = 0,
    .RStickY = 0
};


// Requests data from ESP32 and writes data to data structure
void ReadInputData(){
    i2c_write_blocking(I2C_PORT, ESP32_SLAVE_ADDR, NULL, 0, true);

    int bytesRead = i2c_read_blocking(I2C_PORT, ESP32_SLAVE_ADDR, (uint8_t*)&receivedData, sizeof(receivedData), false);

    if (bytesRead != sizeof(receivedData)) {
        printf("Failed to read data from ESP32. Bytes read: %d\n", bytesRead);
    }
}

bool isJustPressed(const std::string& buttonName, bool currentState) {
    static std::map<std::string, bool> previousStates;

    bool justPressed = currentState && !previousStates[buttonName];
    previousStates[buttonName] = currentState;

    return justPressed;
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
        
        if (isJustPressed("Triangle", receivedData.Triangle)) {animation.CycleGait();}
        
        animation.Strafe();
        
        sleep_ms(1);
    }
}