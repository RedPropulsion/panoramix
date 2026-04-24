cmake_minimum_required(VERSION 3.20.0)

add_library(sound INTERFACE)
target_include_directories(sound INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE prefix>)
target_sources(sound INTERFACE sound.c)