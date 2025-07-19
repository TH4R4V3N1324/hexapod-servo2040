#include "Hexapod.h"

int main() {
    stdio_init_all();
    InitI2C();

    Animation animation;

    sleep_ms(5000);
    animation.Startup();

    while (1) {
        ReadInputData();
        animation.Strafe();
        sleep_ms(1);
    }
}