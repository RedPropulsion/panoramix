cmake_minimum_required(VERSION 3.20.0)

set(BOARD asterics)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(AsterICS)
target_sources(app PRIVATE src/main_led_gem.c)
