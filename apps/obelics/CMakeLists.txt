cmake_minimum_required(VERSION 3.20.0)

set(BOARD nucleo_h723zg)
set(BOARD_FLASH_RUNNER openocd)
set(BOARD_DEBUG_RUNNER openocd)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/sound
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/udp_client
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ObelICS)

target_sources(app PRIVATE
    src/main.c
)

target_link_libraries(app PRIVATE
    panoramix_sound
    panoramix_udp_client
)

target_include_directories(app PRIVATE
    src
)
