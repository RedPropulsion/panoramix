# SPDX-License-Identifier: Apache-2.0

# Parent directory - sources are in u_blox_m10 subdirectory
zephyr_library_named(panoramix_gnss)

# Add Zephyr's GNSS driver source directory for includes
zephyr_include_directories(${ZEPHYR_BASE}/drivers/gnss)

add_subdirectory_ifdef(CONFIG_GNSS_U_BLOX_M10 u_blox_m10)

# Allow empty in case no GNSS drivers are enabled
set_property(TARGET panoramix_gnss PROPERTY ALLOW_EMPTY TRUE)