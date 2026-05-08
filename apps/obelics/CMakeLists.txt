cmake_minimum_required(VERSION 3.20.0)

set(BOARD nucleo_h723zg)
set(BOARD_FLASH_RUNNER openocd)
set(BOARD_DEBUG_RUNNER openocd)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/include_custom_modules.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ObelICS)
target_sources(app PRIVATE
    src/main.c

