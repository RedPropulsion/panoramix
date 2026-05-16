cmake_minimum_required(VERSION 3.20.0)
set(BOARD nucleo_h723zg)
set(BOARD_FLASH_RUNNER openocd)
set(BOARD_DEBUG_RUNNER openocd)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/include_custom_modules.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ObelICS)
# ============ FONT GENERATION ============
set(gen_dir ${CMAKE_BINARY_DIR}/include/generated/)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(
    OUTPUT ${gen_dir}/cfb_font_templeos.h
    COMMAND 
        ${Python3_EXECUTABLE} 
        ${ZEPHYR_BASE}/scripts/build/gen_cfb_font_header.py
        --zephyr-base ${ZEPHYR_BASE}
        --input ${CMAKE_SOURCE_DIR}/fonts/templeos.otf
        --output ${gen_dir}/cfb_font_templeos.h
        --width 8
        --height 8
        -s 8
    DEPENDS ${CMAKE_SOURCE_DIR}/fonts/templeos.otf
    COMMENT "Generating CFB font header from templeos.otf"
)
add_custom_target(generate_cfb_font DEPENDS ${gen_dir}/cfb_font_templeos.h)
add_dependencies(app generate_cfb_font)
# ============ KEY FIX: Add include path for generated headers ============
target_include_directories(app PRIVATE ${gen_dir})
target_sources(app PRIVATE
    src/main.c
)