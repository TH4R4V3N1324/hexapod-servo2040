#include "Hexapod.h"

Animation animation;

void CommandFSM() {
    switch (controlPacket.command) {
        case CMD_SET_GAIT:
            animation.SetGait(static_cast<Gait>(controlPacket.commandArgs[0]));
            break;
        case CMD_SET_MODE:
            animation.SetMode(static_cast<Mode>(controlPacket.commandArgs[0]));
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
        case MODE_NORMAL:
            animation.Normal();
            break;
        case MODE_STRAFE:
            animation.Strafe();
            break;
        case MODE_TILT:
            break;
        case MODE_CONFIG:
            animation.ConfigState();
            break;
        default:
            break;
    }    
}

int main() {
    stdio_init_all();
    InitI2C();
    configManager.loadLegOffsets();
    hexPacket.currentHeight = 120;

    sleep_ms(5000);
    animation.Startup();

    while (1) {
        ReadInputData();
        if(CommandChanged()) {CommandFSM();}
        SendHexData();
        StateFSM();
        sleep_ms(1);
    }
}