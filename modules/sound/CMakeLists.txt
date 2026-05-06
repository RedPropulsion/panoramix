if(CONFIG_LIB_SOUND)
  add_library(panoramix_sound INTERFACE)

  target_include_directories(panoramix_sound INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  zephyr_library()
  zephyr_library_sources(sound.c)

  zephyr_library_link_libraries(panoramix_sound)
  target_link_libraries(panoramix_sound INTERFACE zephyr_interface)
endif()