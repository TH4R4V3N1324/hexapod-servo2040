#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include "hardware/flash.h"
#include "pico/stdlib.h"
#include <cstring>

#define FLASH_TARGET_OFFSET (1024 * 1024) // 1MB

class ConfigManager {
    private:
        static int16_t jointOffsets[6][3];

    public:
        void saveLegOffsets();
        void loadLegOffsets();
        void SetLegConfig(int legNum, int joint, int offset);
        const int16_t* getLegOffsets(int legNum) const;
};

extern ConfigManager configManager;

#endif