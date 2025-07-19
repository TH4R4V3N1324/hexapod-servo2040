#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include "hardware/flash.h"
#include "pico/stdlib.h"
#include <cstring>

#define FLASH_TARGET_OFFSET (1024 * 1024) // 1MB

class ConfigManager {
    private:
        int16_t jointOffsets[6][3];

    public:
        void saveLegOffsets(const int16_t legOffsets[6][3]);
        void loadLegOffsets(int16_t legOffsets[6][3]);
        void SetLegConfig(int legNum, const int16_t newOffsets[3]);
};

#endif