#pragma once

#include <assert.h>
#include <stdbool.h>

#include "main.h"
#include "DWT_Delay.h"

#define FLASH_TIMEOUT_MS 50000UL    // 50 s

// Sectors flash memory id range for specific data types. From 0 to (max - 1). Set range for own use or leave defaults
#define FLASH_SECTOR_5_BYTE_ID_RANGE        14000
#define FLASH_SECTOR_5_HALF_WORD_ID_RANGE   13000
#define FLASH_SECTOR_5_WORD_ID_RANGE        12000
#define FLASH_SECTOR_5_DOUBLE_WORD_ID_RANGE 5000

#define FLASH_SECTOR_6_BYTE_ID_RANGE        14000
#define FLASH_SECTOR_6_HALF_WORD_ID_RANGE   13000
#define FLASH_SECTOR_6_WORD_ID_RANGE        12000
#define FLASH_SECTOR_6_DOUBLE_WORD_ID_RANGE 5000

#define FLASH_SECTOR_7_BYTE_ID_RANGE        14000
#define FLASH_SECTOR_7_HALF_WORD_ID_RANGE   13000
#define FLASH_SECTOR_7_WORD_ID_RANGE        12000
#define FLASH_SECTOR_7_DOUBLE_WORD_ID_RANGE 5000

typedef enum FlashVoltageRange {
    FLASH_VOLTAGE_RANGE_1,  // Device operating range: 1.8V to 2.1V
    FLASH_VOLTAGE_RANGE_2,  // Device operating range: 2.1V to 2.7V
    FLASH_VOLTAGE_RANGE_3,  // Device operating range: 2.7V to 3.6V
    FLASH_VOLTAGE_RANGE_4   // Device operating range: 2.7V to 3.6V + External Vpp
} FlashVoltageRange;

typedef enum FlashSector {
    FLASH_SECTOR_5 = 5, // 128 kb
    FLASH_SECTOR_6 = 6, // 128 kb
    FLASH_SECTOR_7 = 7  // 128 kb
} FlashSector;

typedef enum FlashDataType {
    FLASH_DATA_TYPE_BYTE,        // Program byte (8-bit) at a specified address
    FLASH_DATA_TYPE_HALF_WORD,   // Program a half-word (16-bit) at a specified address
    FLASH_DATA_TYPE_WORD,        // Program a word (32-bit) at a specified address
    FLASH_DATA_TYPE_DOUBLE_WORD, // Program a double word (64-bit) at a specified address
    FLASH_DATA_TYPE_FLOAT,       // Program a float as word (32-bit) at a specified address
    FLASH_DATA_TYPE_DOUBLE       // Program a double as double word (64-bit) at a specified address
} FlashDataType;

void initFlashMemory(FlashVoltageRange voltageRange);
void setFlashMemorySector(FlashSector sectorNumber);
void eraseSectorFlashMemory();

void writeToFlashMemory(uint32_t id, FlashDataType dataType, uint64_t data);
void writeFloatToFlashMemory(uint32_t id, float data);// Float value has been saved to 32-bit area. Must use unique id for preventing data overlapping with 32-bit values
void writeDoubleToFlashMemory(uint32_t id, double data);// Double value has been saved to 64-bit area. Must use unique id for preventing data overlapping with 64-bit values

uint64_t readFromFlashMemory(uint32_t id, FlashDataType dataType);
float readFloatFromFlashMemory(uint32_t id);
double readDoubleFromFlashMemory(uint32_t id);