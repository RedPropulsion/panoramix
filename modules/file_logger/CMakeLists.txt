add_library(panoramix_file_logger INTERFACE)

target_include_directories(panoramix_file_logger INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/fs/fatfs/include
)

zephyr_library()
zephyr_library_sources(file_logger.c)

zephyr_library_link_libraries(ELMFAT)

zephyr_library_link_libraries(panoramix_file_logger)
target_link_libraries(panoramix_file_logger INTERFACE zephyr_interface)