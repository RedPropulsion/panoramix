## Note: this file is included by `zephyr/module.yml`

# Make the `include` directory available
# zephyr_syscall_include_directories(include)
# zephyr_include_directories(include)

cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})