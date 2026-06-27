#include "crc32.h"

#include <pico/critical_section.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t  __not_in_flash("crc3_data")  crc32LookupTable[256] = {};

inline uint32_t crc32TableEntry(uint32_t index) {
    for (unsigned bit = 0; bit < 8; ++bit) {
        index = (index >> 1) ^ (0xEDB88320 & -(index & 1));
    }
    return index;
}

void initCrc32() {
    for (uint32_t i = 0; i < 256; ++i) {
        crc32LookupTable[i] = crc32TableEntry(i);
    }
}

inline static uint32_t __not_in_flash_func(crc32Seeded)(const uint8_t *data, size_t size, const uint32_t seed) {
    uint32_t crc = ~seed;

    while (size--) {
        crc = (crc >> 8) ^ crc32LookupTable[(crc ^ *data++) & 0xff];
    }

    return ~crc;
}

inline static uint32_t __not_in_flash_func(crc32)(const uint8_t *data, size_t size) {
    return crc32Seeded(data, size, 0xEADA2D49);  // 0xA2 seed
}

void __not_in_flash_func(fillOutputReportChecksum)(uint8_t *outputData, size_t len) {
    uint32_t crc = crc32(outputData, len - 4);
    outputData[len - 4] = (crc >> 0) & 0xFF;
    outputData[len - 3] = (crc >> 8) & 0xFF;
    outputData[len - 2] = (crc >> 16) & 0xFF;
    outputData[len - 1] = (crc >> 24) & 0xFF;
}

inline static uint32_t crc32Feature(const uint8_t *data, size_t size) {
    // https://github.com/rafaelvaloto/Dualsense-Multiplatform/blob/main/Source/Private/GCore/Utils/CR32.cpp
    return crc32Seeded(data, size, 0x2060efc3);  // 0x53 seed
}

void fillFeatureReportChecksum(uint8_t *data, const size_t len) {
    uint32_t crc = crc32Feature(data, len - 4);
    data[len - 4] = (crc >> 0) & 0xFF;
    data[len - 3] = (crc >> 8) & 0xFF;
    data[len - 2] = (crc >> 16) & 0xFF;
    data[len - 1] = (crc >> 24) & 0xFF;
}
