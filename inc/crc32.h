#pragma once

#include <stddef.h>
#include <stdint.h>

void initCrc32();
void fillOutputReportChecksum(uint8_t *outputData, size_t len);
void fillFeatureReportChecksum(uint8_t *data, size_t len);
