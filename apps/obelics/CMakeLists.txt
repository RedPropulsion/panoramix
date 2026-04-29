cmake_minimum_required(VERSION 3.20.0)

set(BOARD obelics)
set(BOARD_FLASH_RUNNER openocd)
set(BOARD_DEBUG_RUNNER openocd)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/sound
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/udp_client
)

list(APPEND DTS_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ObelICS)

target_sources(app PRIVATE
    src/main.c
)

target_include_directories(app PRIVATE
    src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/sound
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/udp_client
)