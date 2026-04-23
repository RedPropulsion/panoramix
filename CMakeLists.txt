## Note: this file is included by `zephyr/module.yml`

# Make the `include` directory available
zephyr_syscall_include_directories(include)
zephyr_include_directories(include)

add_subdirectory(drivers)
