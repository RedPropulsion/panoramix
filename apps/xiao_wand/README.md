# Getting Started

in order to build and flash you need to run '''west blobs fetch hal_espressif'''

then you need the xtensa compiler, to do so go in the zephyr sdk installation folder (user/io/zephy-sdk-1.0.1) and run '''./setup.sh -t xtensa-espressif_esp32s3_zephyr-elf'''

then you also need esptool to be installed

per flashare usa '''west flash --reset-type watchdog-reset'''