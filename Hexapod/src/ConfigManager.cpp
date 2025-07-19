#include "ConfigManager.h"

// Save offsets
void ConfigManager::saveLegOffsets(const int16_t legAngleOffsets[6][3]) {
    flash_range_erase(FLASH_TARGET_OFFSET, 4096); // Erase 4KB sector
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t*)legAngleOffsets, sizeof(int16_t) * 6 * 3);
}

// Load offsets
void ConfigManager::loadLegOffsets(int16_t legAngleOffsets[6][3]) {
    const uint8_t* flash_ptr = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(legAngleOffsets, flash_ptr, sizeof(int16_t) * 6 * 3);
}

// Change offsets for leg
void ConfigManager::SetLegConfig(int legNum, const int16_t newOffsets[3]) {
    jointOffsets[legNum][0] = newOffsets[0];
    jointOffsets[legNum][1] = newOffsets[1];
    jointOffsets[legNum][2] = newOffsets[2];

    saveLegOffsets(jointOffsets);
}