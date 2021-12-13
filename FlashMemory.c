#include "FlashMemory.h"

#define FLASH_SECTOR_SIZE_128K_BYTES 128000 // 128 kb

#define FLASH_SECTOR_5_START_ADDRESS 0x08020000
#define FLASH_SECTOR_5_END_ADDRESS 0x0803FFFF

#define FLASH_SECTOR_6_START_ADDRESS 0x08040000
#define FLASH_SECTOR_6_END_ADDRESS 0x0805FFFF

#define FLASH_SECTOR_7_START_ADDRESS 0x08060000
#define FLASH_SECTOR_7_END_ADDRESS 0x0807FFFF

// Calculate specific flash areas by type size in bytes
#define FLASH_SECTOR_5_BYTE_AREA_SIZE        (FLASH_SECTOR_5_BYTE_ID_RANGE * sizeof(uint8_t))
#define FLASH_SECTOR_5_HALF_WORD_AREA_SIZE   (FLASH_SECTOR_5_HALF_WORD_ID_RANGE * sizeof(uint16_t))
#define FLASH_SECTOR_5_WORD_AREA_SIZE        (FLASH_SECTOR_5_WORD_ID_RANGE * sizeof(uint32_t))
#define FLASH_SECTOR_5_DOUBLE_WORD_AREA_SIZE (FLASH_SECTOR_5_DOUBLE_WORD_ID_RANGE * sizeof(uint64_t))

#define FLASH_SECTOR_6_BYTE_AREA_SIZE        (FLASH_SECTOR_6_BYTE_ID_RANGE * sizeof(uint8_t))
#define FLASH_SECTOR_6_HALF_WORD_AREA_SIZE   (FLASH_SECTOR_6_HALF_WORD_ID_RANGE * sizeof(uint16_t))
#define FLASH_SECTOR_6_WORD_AREA_SIZE        (FLASH_SECTOR_6_WORD_ID_RANGE * sizeof(uint32_t))
#define FLASH_SECTOR_6_DOUBLE_WORD_AREA_SIZE (FLASH_SECTOR_6_DOUBLE_WORD_ID_RANGE * sizeof(uint64_t))

#define FLASH_SECTOR_7_BYTE_AREA_SIZE        (FLASH_SECTOR_7_BYTE_ID_RANGE * sizeof(uint8_t))
#define FLASH_SECTOR_7_HALF_WORD_AREA_SIZE   (FLASH_SECTOR_7_HALF_WORD_ID_RANGE * sizeof(uint16_t))
#define FLASH_SECTOR_7_WORD_AREA_SIZE        (FLASH_SECTOR_7_WORD_ID_RANGE * sizeof(uint32_t))
#define FLASH_SECTOR_7_DOUBLE_WORD_AREA_SIZE (FLASH_SECTOR_7_DOUBLE_WORD_ID_RANGE * sizeof(uint64_t))

// Check total area size
#define FLASH_SECTOR_5_TOTAL_AREA_SIZE ( \
            FLASH_SECTOR_5_BYTE_AREA_SIZE +          \
            FLASH_SECTOR_5_HALF_WORD_AREA_SIZE +     \
            FLASH_SECTOR_5_WORD_AREA_SIZE +          \
            FLASH_SECTOR_5_DOUBLE_WORD_AREA_SIZE)
static_assert(FLASH_SECTOR_5_TOTAL_AREA_SIZE <= FLASH_SECTOR_SIZE_128K_BYTES, "Flash sector 5 memory overflow");

#define FLASH_SECTOR_6_TOTAL_AREA_SIZE ( \
            FLASH_SECTOR_6_BYTE_AREA_SIZE +          \
            FLASH_SECTOR_6_HALF_WORD_AREA_SIZE +     \
            FLASH_SECTOR_6_WORD_AREA_SIZE +          \
            FLASH_SECTOR_6_DOUBLE_WORD_AREA_SIZE)
static_assert(FLASH_SECTOR_6_TOTAL_AREA_SIZE <= FLASH_SECTOR_SIZE_128K_BYTES, "Flash sector 6 memory overflow");

#define FLASH_SECTOR_7_TOTAL_AREA_SIZE ( \
            FLASH_SECTOR_7_BYTE_AREA_SIZE +          \
            FLASH_SECTOR_7_HALF_WORD_AREA_SIZE +     \
            FLASH_SECTOR_7_WORD_AREA_SIZE +          \
            FLASH_SECTOR_7_DOUBLE_WORD_AREA_SIZE)
static_assert(FLASH_SECTOR_7_TOTAL_AREA_SIZE <= FLASH_SECTOR_SIZE_128K_BYTES, "Flash sector 7 memory overflow");

// Calculate flash addresses by area size
#define CALCULATE_FLASH_END_ADDRESS(startAddress, areaSize) ((startAddress) + (areaSize))

// Sector 5 area address calculation
#define FLASH_SECTOR_5_BYTE_AREA_START_ADDRESS FLASH_SECTOR_5_START_ADDRESS
#define FLASH_SECTOR_5_BYTE_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_5_BYTE_AREA_START_ADDRESS, FLASH_SECTOR_5_BYTE_AREA_SIZE)

#define FLASH_SECTOR_5_HALF_WORD_AREA_START_ADDRESS FLASH_SECTOR_5_BYTE_AREA_END_ADDRESS
#define FLASH_SECTOR_5_HALF_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_5_HALF_WORD_AREA_START_ADDRESS, FLASH_SECTOR_5_HALF_WORD_AREA_SIZE)

#define FLASH_SECTOR_5_WORD_AREA_START_ADDRESS FLASH_SECTOR_5_HALF_WORD_AREA_END_ADDRESS
#define FLASH_SECTOR_5_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_5_WORD_AREA_START_ADDRESS, FLASH_SECTOR_5_WORD_AREA_SIZE)

#define FLASH_SECTOR_5_DOUBLE_WORD_AREA_START_ADDRESS FLASH_SECTOR_5_WORD_AREA_END_ADDRESS
#define FLASH_SECTOR_5_DOUBLE_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_5_DOUBLE_WORD_AREA_START_ADDRESS, FLASH_SECTOR_5_DOUBLE_WORD_AREA_SIZE)

// Sector 6 area address calculation
#define FLASH_SECTOR_6_BYTE_AREA_START_ADDRESS FLASH_SECTOR_6_START_ADDRESS
#define FLASH_SECTOR_6_BYTE_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_6_BYTE_AREA_START_ADDRESS, FLASH_SECTOR_6_BYTE_AREA_SIZE)

#define FLASH_SECTOR_6_HALF_WORD_AREA_START_ADDRESS FLASH_SECTOR_6_BYTE_AREA_END_ADDRESS
#define FLASH_SECTOR_6_HALF_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_6_HALF_WORD_AREA_START_ADDRESS, FLASH_SECTOR_6_HALF_WORD_AREA_SIZE)

#define FLASH_SECTOR_6_WORD_AREA_START_ADDRESS FLASH_SECTOR_6_HALF_WORD_AREA_END_ADDRESS
#define FLASH_SECTOR_6_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_6_WORD_AREA_START_ADDRESS, FLASH_SECTOR_6_WORD_AREA_SIZE)

#define FLASH_SECTOR_6_DOUBLE_WORD_AREA_START_ADDRESS FLASH_SECTOR_6_WORD_AREA_END_ADDRESS
#define FLASH_SECTOR_6_DOUBLE_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_6_DOUBLE_WORD_AREA_START_ADDRESS, FLASH_SECTOR_6_DOUBLE_WORD_AREA_SIZE)

// Sector 7 area address calculation
#define FLASH_SECTOR_7_BYTE_AREA_START_ADDRESS FLASH_SECTOR_7_START_ADDRESS
#define FLASH_SECTOR_7_BYTE_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_7_BYTE_AREA_START_ADDRESS, FLASH_SECTOR_7_BYTE_AREA_SIZE)

#define FLASH_SECTOR_7_HALF_WORD_AREA_START_ADDRESS FLASH_SECTOR_7_BYTE_AREA_END_ADDRESS
#define FLASH_SECTOR_7_HALF_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_7_HALF_WORD_AREA_START_ADDRESS, FLASH_SECTOR_7_HALF_WORD_AREA_SIZE)

#define FLASH_SECTOR_7_WORD_AREA_START_ADDRESS FLASH_SECTOR_7_HALF_WORD_AREA_END_ADDRESS
#define FLASH_SECTOR_7_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_7_WORD_AREA_START_ADDRESS, FLASH_SECTOR_7_WORD_AREA_SIZE)

#define FLASH_SECTOR_7_DOUBLE_WORD_AREA_START_ADDRESS FLASH_SECTOR_7_WORD_AREA_END_ADDRESS
#define FLASH_SECTOR_7_DOUBLE_WORD_AREA_END_ADDRESS   CALCULATE_FLASH_END_ADDRESS(FLASH_SECTOR_7_DOUBLE_WORD_AREA_START_ADDRESS, FLASH_SECTOR_7_DOUBLE_WORD_AREA_SIZE)

// Check sectors end address
static_assert(FLASH_SECTOR_5_DOUBLE_WORD_AREA_END_ADDRESS <= FLASH_SECTOR_5_END_ADDRESS, "Flash sector 5 address is out of bounds");
static_assert(FLASH_SECTOR_6_DOUBLE_WORD_AREA_END_ADDRESS <= FLASH_SECTOR_6_END_ADDRESS, "Flash sector 6 address is out of bounds");
static_assert(FLASH_SECTOR_7_DOUBLE_WORD_AREA_END_ADDRESS <= FLASH_SECTOR_7_END_ADDRESS, "Flash sector 7 address is out of bounds");


// FLASH_Program_Parallelism FLASH Program Parallelism
#define FLASH_POINTER_SIZE_BYTE         0x00000000U
#define FLASH_POINTER_SIZE_HALF_WORD    0x00000100U
#define FLASH_POINTER_SIZE_WORD         0x00000200U
#define FLASH_POINTER_SIZE_DOUBLE_WORD  0x00000300U

//FLASH_Keys FLASH Keys
#define FLASH_KEY1  0x45670123
#define FLASH_KEY2  0xCDEF89AB

#define INVALID_FLASH_ADDRESS 0

union FloatConverter {
    float floatValue;
    uint32_t u32Value;
} floatConverter = { .u32Value = 0 };

union DoubleConverter {
    double doubleValue;
    uint64_t u64Value;
} doubleConverter = { .u64Value = 0 };

static FlashVoltageRange flashMemoryVoltageRange;
static FlashSector flashMemorySector;

static bool programFlashMemory(FlashDataType dataType, uint32_t address, uint64_t data);
static void clearSectorFlashMemory(uint32_t sector, FlashVoltageRange voltageRange);
static bool unlockFlashMemory();
static void lockFlashMemory();

static bool waitForOperationFlashMemory();
static void programByteToFlash(uint32_t address, uint8_t data);
static void programHalfWordToFlash(uint32_t address, uint16_t data);
static void programWordToFlash(uint32_t address, uint32_t data);
static void programDoubleWordToFlash(uint32_t address, uint64_t data);

static uint32_t getSectorAddressById(uint32_t id, FlashSector sectorNumber, FlashDataType dataType);
static uint32_t resolveSectorByteAreaAddress(uint32_t id, FlashSector sectorNumber);
static uint32_t resolveSectorHalfWordAreaAddress(uint32_t id, FlashSector sectorNumber);
static uint32_t resolveSectorWordAreaAddress(uint32_t id, FlashSector sectorNumber);
static uint32_t resolveSectorDoubleWordAreaAddress(uint32_t id, FlashSector sectorNumber);
static inline bool isFlashIdRangeValid(uint32_t id, uint32_t sectorIdRange);


void initFlashMemory(FlashVoltageRange voltageRange) {
    flashMemoryVoltageRange = voltageRange;
    dwtDelayInit();
}

void setFlashMemorySector(FlashSector sectorNumber) {
    flashMemorySector = sectorNumber;
}

void eraseSectorFlashMemory() {
    if (unlockFlashMemory()) {
        clearSectorFlashMemory(flashMemorySector, flashMemoryVoltageRange);
        lockFlashMemory();
    }
}

void writeToFlashMemory(uint32_t id, FlashDataType dataType, uint64_t data) {
    if (unlockFlashMemory()) {
        uint32_t flashAddress = getSectorAddressById(id, flashMemorySector, dataType);
        programFlashMemory(dataType, flashAddress, data);
        lockFlashMemory();
    }
}

void writeFloatToFlashMemory(uint32_t id, float data) {
    if (unlockFlashMemory()) {
        uint32_t flashAddress = getSectorAddressById(id, flashMemorySector, FLASH_DATA_TYPE_FLOAT);
        floatConverter.floatValue = data;
        programFlashMemory(FLASH_DATA_TYPE_FLOAT, flashAddress, floatConverter.u32Value);
        lockFlashMemory();
    }
}

void writeDoubleToFlashMemory(uint32_t id, double data) {
    if (unlockFlashMemory()) {
        uint32_t flashAddress = getSectorAddressById(id, flashMemorySector, FLASH_DATA_TYPE_DOUBLE);
        doubleConverter.doubleValue = data;
        programFlashMemory(FLASH_DATA_TYPE_DOUBLE, flashAddress, doubleConverter.u64Value);
        lockFlashMemory();
    }
}

uint64_t readFromFlashMemory(uint32_t id, FlashDataType dataType) {
    uint32_t address = getSectorAddressById(id, flashMemorySector, dataType);
    switch (dataType) {
        case FLASH_DATA_TYPE_BYTE:
            return *(uint8_t *) address;
        case FLASH_DATA_TYPE_HALF_WORD:
            return *(uint16_t *) address;
        case FLASH_DATA_TYPE_WORD:
        case FLASH_DATA_TYPE_FLOAT:
            return *(uint32_t *) address;
        case FLASH_DATA_TYPE_DOUBLE_WORD:
        case FLASH_DATA_TYPE_DOUBLE:
            return *(uint64_t *) address;
        default:
            return 0;
    }
}

float readFloatFromFlashMemory(uint32_t id) {
    floatConverter.u32Value = readFromFlashMemory(id, FLASH_DATA_TYPE_FLOAT);
    return floatConverter.floatValue;
}

double readDoubleFromFlashMemory(uint32_t id) {
    doubleConverter.u64Value = readFromFlashMemory(id, FLASH_DATA_TYPE_DOUBLE);
    return doubleConverter.doubleValue;
}

static bool programFlashMemory(FlashDataType dataType, uint32_t address, uint64_t data) {
    if (address == INVALID_FLASH_ADDRESS) return false;
    bool isFlashMemoryReady = waitForOperationFlashMemory();
    if (!isFlashMemoryReady) return isFlashMemoryReady;

    switch (dataType) {  // Program byte, half word, word or double word at a specified address
        case FLASH_DATA_TYPE_BYTE:
            programByteToFlash(address, (uint8_t) data);
            break;
        case FLASH_DATA_TYPE_HALF_WORD:
            programHalfWordToFlash(address, (uint16_t) data);
            break;
        case FLASH_DATA_TYPE_WORD:
        case FLASH_DATA_TYPE_FLOAT:
            programWordToFlash(address, (uint32_t) data);
            break;
        case FLASH_DATA_TYPE_DOUBLE_WORD:
        case FLASH_DATA_TYPE_DOUBLE:
            programDoubleWordToFlash(address, data);
        default:
            return false;
    }

    isFlashMemoryReady = waitForOperationFlashMemory();
    FLASH->CR &= (~FLASH_CR_PG);    // If the program operation is completed, disable the PG Bit
    return isFlashMemoryReady;
}

static void clearSectorFlashMemory(uint32_t sector, FlashVoltageRange voltageRange) {
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);

    switch (voltageRange) {
        case FLASH_VOLTAGE_RANGE_1:
            FLASH->CR |= FLASH_POINTER_SIZE_BYTE;
            break;
        case FLASH_VOLTAGE_RANGE_2:
            FLASH->CR |= FLASH_POINTER_SIZE_HALF_WORD;
            break;
        case FLASH_VOLTAGE_RANGE_3:
            FLASH->CR |= FLASH_POINTER_SIZE_WORD;
            break;
        case FLASH_VOLTAGE_RANGE_4:
            FLASH->CR |= FLASH_POINTER_SIZE_DOUBLE_WORD;
            break;
        default:
            return;
    }
    /* If the previous operation is completed, proceed to erase the sector */
    CLEAR_BIT(FLASH->CR, FLASH_CR_SNB);
    FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
}

static bool unlockFlashMemory() { // Unlock the FLASH control register access
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) {  // Authorize the FLASH Registers access
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK)) {  // Verify Flash is unlocked
            return false;
        }
    }
    return true;
}

static void lockFlashMemory() { // Locks the FLASH control register access
    FLASH->CR |= FLASH_CR_LOCK;     // Set the LOCK Bit to lock the FLASH Registers access
}

static bool waitForOperationFlashMemory() {
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
       Even if the FLASH operation fails, the BUSY flag will be reset and an error
       flag will be set */
    uint32_t startTime = currentMilliSeconds();

    while (READ_BIT(FLASH->SR, FLASH_SR_BSY) == SET) {
        if ((currentMilliSeconds() - startTime) >= FLASH_TIMEOUT_MS) {
            return false;
        }
    }
    CLEAR_BIT(FLASH->SR, FLASH_SR_EOP); // Clear FLASH End of Operation pending bit
    return true;
}

//Program byte (8-bit) at a specified address. This function must be used when the device voltage range is from 1.8V to 3.6V.
static void programByteToFlash(uint32_t address, uint8_t data) {
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);// If the previous operation is completed, proceed to program the new data
    FLASH->CR |= FLASH_POINTER_SIZE_BYTE;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint8_t *) address = data;
}

//Program a half-word (16-bit) at a specified address. This function must be used when the device voltage range is from 2.1V to 3.6V.
static void programHalfWordToFlash(uint32_t address, uint16_t data) {
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);// If the previous operation is completed, proceed to program the new data
    FLASH->CR |= FLASH_POINTER_SIZE_HALF_WORD;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint16_t *) address = data;
}

// Program word (32-bit) at a specified address. This function must be used when the device voltage range is from 2.7V to 3.6V.
static void programWordToFlash(uint32_t address, uint32_t data) {
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);// If the previous operation is completed, proceed to program the new data
    FLASH->CR |= FLASH_POINTER_SIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint32_t *) address = data;
}

// Program a double word (64-bit) at a specified address.
// This function must be used when the device voltage range is from 2.7V to 3.6V and Vpp in the range 7V to 9V.
static void programDoubleWordToFlash(uint32_t address, uint64_t data) {
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);// If the previous operation is completed, proceed to program the new data
    FLASH->CR |= FLASH_POINTER_SIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint64_t *) address = data;
}

static uint32_t getSectorAddressById(uint32_t id, FlashSector sectorNumber, FlashDataType dataType) {
    uint32_t startAddress;
    uint8_t dataTypeLength;

    switch (dataType) {
        case FLASH_DATA_TYPE_BYTE:
            dataTypeLength = sizeof(uint8_t);
            startAddress = resolveSectorByteAreaAddress(id, sectorNumber);
            break;
        case FLASH_DATA_TYPE_HALF_WORD:
            dataTypeLength = sizeof(uint16_t);
            startAddress = resolveSectorHalfWordAreaAddress(id, sectorNumber);
            break;
        case FLASH_DATA_TYPE_WORD:
        case FLASH_DATA_TYPE_FLOAT:
            dataTypeLength = sizeof(uint32_t);
            startAddress = resolveSectorWordAreaAddress(id, sectorNumber);
            break;
        case FLASH_DATA_TYPE_DOUBLE_WORD:
        case FLASH_DATA_TYPE_DOUBLE:
            dataTypeLength = sizeof(uint64_t);
            startAddress = resolveSectorDoubleWordAreaAddress(id, sectorNumber);
            break;
        default:
            return INVALID_FLASH_ADDRESS;
    }

    return (startAddress != INVALID_FLASH_ADDRESS) ? (startAddress + (id * dataTypeLength)) : startAddress;
}

static uint32_t resolveSectorByteAreaAddress(uint32_t id, FlashSector sectorNumber) {
    if (sectorNumber == FLASH_SECTOR_7 && isFlashIdRangeValid(id, FLASH_SECTOR_7_BYTE_ID_RANGE)) {
        return FLASH_SECTOR_7_BYTE_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_6 && isFlashIdRangeValid(id, FLASH_SECTOR_6_BYTE_ID_RANGE)) {
        return FLASH_SECTOR_6_BYTE_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_5 && isFlashIdRangeValid(id, FLASH_SECTOR_5_BYTE_ID_RANGE)) {
        return FLASH_SECTOR_5_BYTE_AREA_START_ADDRESS;
    }
    return INVALID_FLASH_ADDRESS;
}

static uint32_t resolveSectorHalfWordAreaAddress(uint32_t id, FlashSector sectorNumber) {
    if (sectorNumber == FLASH_SECTOR_7 && isFlashIdRangeValid(id, FLASH_SECTOR_7_HALF_WORD_ID_RANGE)) {
        return FLASH_SECTOR_7_HALF_WORD_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_6 && isFlashIdRangeValid(id, FLASH_SECTOR_6_HALF_WORD_ID_RANGE)) {
        return FLASH_SECTOR_6_HALF_WORD_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_5 && isFlashIdRangeValid(id, FLASH_SECTOR_5_HALF_WORD_ID_RANGE)) {
        return FLASH_SECTOR_5_HALF_WORD_AREA_START_ADDRESS;
    }
    return INVALID_FLASH_ADDRESS;
}

static uint32_t resolveSectorWordAreaAddress(uint32_t id, FlashSector sectorNumber) {
    if (sectorNumber == FLASH_SECTOR_7 && isFlashIdRangeValid(id, FLASH_SECTOR_7_WORD_ID_RANGE)) {
        return FLASH_SECTOR_7_WORD_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_6 && isFlashIdRangeValid(id, FLASH_SECTOR_6_WORD_ID_RANGE)) {
        return FLASH_SECTOR_6_WORD_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_5 && isFlashIdRangeValid(id, FLASH_SECTOR_5_WORD_ID_RANGE)) {
        return FLASH_SECTOR_5_WORD_AREA_START_ADDRESS;
    }
    return INVALID_FLASH_ADDRESS;
}

static uint32_t resolveSectorDoubleWordAreaAddress(uint32_t id, FlashSector sectorNumber) {
    if (sectorNumber == FLASH_SECTOR_7 && isFlashIdRangeValid(id, FLASH_SECTOR_7_DOUBLE_WORD_ID_RANGE)) {
        return FLASH_SECTOR_7_DOUBLE_WORD_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_6 && isFlashIdRangeValid(id, FLASH_SECTOR_6_DOUBLE_WORD_ID_RANGE)) {
        return FLASH_SECTOR_6_DOUBLE_WORD_AREA_START_ADDRESS;
    } else if (sectorNumber == FLASH_SECTOR_5 && isFlashIdRangeValid(id, FLASH_SECTOR_5_DOUBLE_WORD_ID_RANGE)) {
        return FLASH_SECTOR_5_DOUBLE_WORD_AREA_START_ADDRESS;
    }
    return INVALID_FLASH_ADDRESS;
}

static inline bool isFlashIdRangeValid(uint32_t id, uint32_t sectorIdRange) {
    return (id < sectorIdRange && sectorIdRange > 0);
}