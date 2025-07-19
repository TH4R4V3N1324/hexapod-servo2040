#include "Hexapod.h"

Animation animation;

void commandFSM() {
    switch (controlPacket.command) {
        case CMD_SET_GAIT:
            break;
        case CMD_SET_MODE:
            break;
        case CMD_ENTER_CONFIG:
            break;
        case CMD_SET_CONFIG:
            break;
        case CMD_HOME_STANCE:
            break;
        case CMD_REQUEST_CONFIG:
            break;
        default:
            break;
    }
}

void StateFSM() {
    switch (animation.currentMode) {
        case Animation::MODE_NORMAL:
            break;
        case Animation::MODE_STRAFE:
            break;
        case Animation::MODE_TILT:
            break;
        case Animation::MODE_CONFIG:
            break;
        default:
            break;
    }    
}

int main() {
    stdio_init_all();
    InitI2C();

    sleep_ms(5000);
    animation.Startup();

    while (1) {
        ReadInputData();
        animation.Strafe();
        sleep_ms(1);
    }
}