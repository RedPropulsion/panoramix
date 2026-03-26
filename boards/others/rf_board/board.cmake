## Force a specific OpenOCD config as auto-detection fails
board_runner_args(openocd
    "--cmd-pre-init" "source [find interface/stlink.cfg]"
    "--cmd-pre-init" "transport select hla_swd"
    "--cmd-pre-init" "adapter speed 1000"
    "--cmd-pre-init" "source [find target/stm32l4x.cfg]"
    "--cmd-pre-init" "reset_config srst_only srst_nogate"
    "--cmd-pre-init" "set WORKAREASIZE 0x8000"
    "--cmd-pre-init" "gdb_report_data_abort enable"
)

    # "--cmd-pre-init" "adapter driver st-link"
    # "--cmd-pre-init" "transport select dapdirect_swd"

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
