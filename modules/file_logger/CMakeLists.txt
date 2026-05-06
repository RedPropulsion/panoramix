zephyr_library()

zephyr_library_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/../../../modules/fs/fatfs/include
)

zephyr_library_link_libraries(ELMFAT)

zephyr_library_sources(file_logger.c)