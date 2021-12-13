# FlashMemory

**STM32** Low Layer(LL) library. Flash memory is a non-volatile storage medium that can be electrically erased and
reprogrammed.

**At now only STM32F4 is supported**\
***NOTE: This is NOT an EEPROM library. Data can't be overwritten. Only after memory erase***

<img src="https://github.com/ximtech/FlashMemory/blob/main/example/flash_map.PNG" alt="image" width="300"/>

### Features

- Id over memory address mapping
- Auto type scale for id
- Works with all standard data types 
- Easy to use and add to project
- No HAL dependency

### Add as CPM project dependency

How to add CPM to the project, check the [link](https://github.com/cpm-cmake/CPM.cmake)

```cmake
CPMAddPackage(
        NAME FlashMemory
        GITHUB_REPOSITORY ximtech/FlashMemory
        GIT_TAG origin/main)
```

### Project configuration

```cmake
include_directories(${includes}
        ${FLASH_MEMORY_DIRECTORY})   # source directories

file(GLOB_RECURSE SOURCES ${sources}
        ${FLASH_MEMORY_SOURCES})    # source files
```

Then Build -> Clean -> Rebuild Project

## Usage

- Usage example: [link](https://github.com/ximtech/FlashMemory/blob/main/example/example.c)
