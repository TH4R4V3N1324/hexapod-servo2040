#include "Hexapod.h"

Animation animation;
ConfigManager configManager;

void CommandFSM() {
    switch (controlPacket.command) {
        case CMD_SET_GAIT:
            animation.SetGait(static_cast<Animation::Gait>(controlPacket.commandArgs[1]));
            break;
        case CMD_SET_MODE:
            animation.SetMode(static_cast<Animation::Mode>(controlPacket.commandArgs[1]));
            break;
        case CMD_SET_CONFIG:
            configManager.SetLegConfig(controlPacket.commandArgs[0], controlPacket.commandArgs[1], controlPacket.commandArgs[2]);
            break;
        case CMD_HOME_STANCE:
            animation.returnToStart();
            break;
        case CMD_REQUEST_CONFIG:
            memcpy(hexPacket.legConfigs, configManager.getLegOffsets(controlPacket.commandArgs[0]), sizeof(int16_t) * 3);
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
            animation.Strafe();
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
    configManager.loadLegOffsets();

    sleep_ms(5000);
    animation.Startup();

    while (1) {
        ReadInputData();
        CommandFSM();
        StateFSM();
        sleep_ms(1);
    }
}