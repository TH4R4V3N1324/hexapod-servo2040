
#include <cstdio>
#include "ConfigManager.h"

ConfigManager configManager;

// Definition of static member
int16_t ConfigManager::jointOffsets[6][3] = {};

// Save offsets
void ConfigManager::saveLegOffsets() {
    flash_range_erase(FLASH_TARGET_OFFSET, 4096); // Erase 4KB sector
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t*)jointOffsets, sizeof(int16_t) * 6 * 3);
}

// Load offsets
void ConfigManager::loadLegOffsets() {
    const uint8_t* flash_ptr = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(jointOffsets, flash_ptr, sizeof(int16_t) * 6 * 3);
}

// Change offsets for leg
void ConfigManager::SetLegConfig(int legNum, int joint, int offset) {
    // Clamp offset to safe range
    if (offset > 60) offset = 60;
    if (offset < -60) offset = -60;
    jointOffsets[legNum][joint] = offset;
    saveLegOffsets();
    // Optional: print for debug
    printf("[ConfigManager] SetLegConfig: leg %d, joint %d, offset %d\n", legNum, joint, offset);
}

// Returns the offsets for a given leg
const int16_t* ConfigManager::getLegOffsets(int legNum) const {
    return jointOffsets[legNum];
}