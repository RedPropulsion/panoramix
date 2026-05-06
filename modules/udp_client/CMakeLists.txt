if(CONFIG_UDP_CLIENT)
  add_library(panoramix_udp_client INTERFACE)

  target_include_directories(panoramix_udp_client INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  zephyr_library()
  zephyr_library_sources(udp_client.c)

  zephyr_library_link_libraries(panoramix_udp_client)
  target_link_libraries(panoramix_udp_client INTERFACE zephyr_interface)
endif()