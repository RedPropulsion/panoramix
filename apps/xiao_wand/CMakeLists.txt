cmake_minimum_required(VERSION 3.20.0)

set(BOARD xiao_esp32s3/esp32s3/procpu)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xiao_wand)

target_sources(app PRIVATE src/main.c)