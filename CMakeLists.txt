cmake_minimum_required(VERSION 3.20)
project(FLASH_MEMORY C)

set(CMAKE_C_STANDARD 99)

include(cmake/CPM.cmake)

CPMAddPackage(
        NAME DWTDelay
        GITHUB_REPOSITORY ximtech/DWTDelay
        GIT_TAG origin/main)

set(FLASH_MEMORY_DIRECTORY
        ${DWT_DELAY_DIRECTORY}
        ${FLASH_MEMORY_SOURCE_DIR}
        ${FLASH_MEMORY_SOURCE_DIR}/include
        CACHE STRING "Flash Memory directories include to the main project" FORCE)

set(FLASH_MEMORY_SOURCES
        ${DWT_DELAY_SOURCES}
        ${FLASH_MEMORY_SOURCE_DIR}/FlashMemory.c
        ${FLASH_MEMORY_SOURCE_DIR}/include/FlashMemory.h
        CACHE STRING "Flash Memory source files include to the main project" FORCE)