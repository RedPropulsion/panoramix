# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# Add Zephyr's GNSS driver source directory for includes
zephyr_include_directories(${ZEPHYR_BASE}/drivers/gnss)

zephyr_library_sources_ifdef(CONFIG_GNSS_U_BLOX_M10 gnss_u_blox_m10.c gnss_u_blox_m10_i2c.c)