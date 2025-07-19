#include "ConfigManager.h"

// Definition of static member
int16_t ConfigManager::jointOffsets[6][3] = {};

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
void ConfigManager::SetLegConfig(int legNum, int joint, int offset) {
    jointOffsets[legNum][joint] = offset;
    saveLegOffsets(jointOffsets);
}

// Returns the offsets for a given leg
const int16_t* ConfigManager::getLegOffsets(int legNum) const {
    return jointOffsets[legNum];
}