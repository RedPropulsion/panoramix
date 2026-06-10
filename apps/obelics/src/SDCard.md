# SD Card (SDMMC) Configuration for Obelics

## Overview

This document explains the SD card configuration for the Obelics board using the STM32H723's SDMMC peripheral.

## Hardware Configuration

### Nucleo H723ZG SDMMC1 Pins

| Signal | Pin   | Function         |
|--------|-------|------------------|
| SDMMC1_D0 | PC8  | Data bit 0      |
| SDMMC1_D1 | PC9  | Data bit 1      |
| SDMMC1_D2 | PC10 | Data bit 2      |
| SDMMC1_D3 | PC11 | Data bit 3 (CS) |
| SDMMC1_CK | PC12 | Clock          |
| SDMMC1_CMD | PD2 | Command        |

## Clock Configuration

### Available Clock Sources

The STM32H723 has several clock sources for SDMMC:

1. **PLL1_Q** - Default, typically 137.5MHz
2. **HSI48** - Dedicated 48MHz internal oscillator
3. **PLL2_Q** - Can be configured for 48MHz

### Internal Clock Divider

The SDMMC has an internal divider used to generate the SD clock:

```
SD_clock = input_clock / (clk_div + 2)
```

For 48MHz SD clock, with input at 137.5MHz: `clk_div = 3` gives 137.5/3 ≈ 45.8MHz (valid for SD spec 40-50MHz)

### Clock Configuration in Device Tree

```dts
&sdmmc1 {
    disk-name = "SD";
    status = "okay";

    clk-div = <3>;  /* Internal divider */
    pinctrl-0 = <&sdmmc1_ck_pc12 &sdmmc1_cmd_pd2 
                 &sdmmc1_d0_pc8 &sdmmc1_d1_pc9 
                 &sdmmc1_d2_pc10 &sdmmc1_d3_pc11>;
    pinctrl-names = "default";
};
```

### Using HSI48 (Alternative)

```dts
/* Enable HSI48 oscillator */
&clk_hsi48 {
    status = "okay";
};

/* Use HSI48 directly */
&sdmmc1 {
    clocks = <&rcc STM32_CLOCK(AHB3, 16)>,
             <&rcc STM32_SRC_HSI48 SDMMC_SEL(0)>;
    /* No internal divider needed */
};
```

## Kconfig Settings

Required configurations in `prj.conf`:

```kconfig
# Filesystem support
CONFIG_FILE_SYSTEM=y
CONFIG_FILE_SYSTEM_LIB_LINK=y

# Disk access
CONFIG_DISK_ACCESS=y
CONFIG_DISK_DRIVER_SDMMC=y

# SDMMC driver
CONFIG_SDMMC_STM32=y

# FAT filesystem
CONFIG_FAT_FILESYSTEM_ELM=y

# Optional: Disable clock rate check (if needed)
# CONFIG_SDMMC_STM32_CLOCK_CHECK=n
```

## How SDMMC Initialization Works

### SD Card Initialization Sequence

1. **CMD0** (GO_IDLE_STATE) - Reset card
2. **CMD8** (SEND_IF_COND) - Check for SD 2.0+ support
3. **ACMD41** (SD_SEND_OP_COND) - Initialize, request SDHC
4. **CMD2** (ALL_SEND_CID) - Get card ID
5. **CMD3** (SEND_RELATIVE_ADDR) - Get RCA
6. **CMD7** (SELECT_CARD) - Select card for transfer
7. **CMD16** (SET_BLOCKLEN) - Set block size

### STM32 HAL Functions Used

- `HAL_SD_Init()` - Initialize SDMMC peripheral
- `HAL_SD_ReadBlocks_DMA()` - Read with DMA
- `HAL_SD_WriteBlocks_DMA()` - Write with DMA

## DMA Configuration

### Default (with DMA)

The driver automatically uses DMA if the `dmas` property is present:

```dts
&sdmmc1 {
    /* DMA is auto-configured if 'dmas' property exists */
};
```

### Interrupt Mode (no DMA)

To use interrupt mode instead of DMA:

```dts
&sdmmc1 {
    /delete-property/ dmas;
};
```

Note: DMA mode is faster but can cause MPU faults if misconfigured.

## Common Issues and Solutions

### 1. Clock Not 48MHz Error

**Symptom:** `SDMMC Clock is not 48MHz`

**Cause:** The clock driver can only return bus clock rates, not PLL source clocks.

**Solution:** Disable the check:
```kconfig
CONFIG_SDMMC_STM32_CLOCK_CHECK=n
```

### 2. MPU Fault / Memory Access Violation

**Symptom:** `MPU FAULT ... Data Access Violation`

**Cause:** DMA buffer misconfiguration

**Solution:** Use interrupt mode (see above)

### 3. CMD7 Timeout

**Symptom:** Card detected but can't initialize

**Causes:**
- Card not properly inserted
- Wrong voltage
- Clock too fast during init

**Checks:**
- ***Verify card is inserted properly*** REALLY DO THIS I WAS GOING FUCKING CRAZY FOR 6+ HOURS OVER THIS
- Try interrupt mode instead of DMA

### 4. Dependencies Missing

**Symptom:** `undefined reference to fs_mount`

**Missing configs:**
- `CONFIG_FILE_SYSTEM=y`
- `CONFIG_FILE_SYSTEM_LIB_LINK=y`

## References

- STM32H723 Reference Manual (RM0468)
- SD Association Physical Layer Simplified Specification
- Zephyr SDMMC Driver: `zephyr/drivers/disk/sdmmc_stm32.c`
- STM32 Clock Driver: `zephyr/drivers/clock_control/clock_stm32_ll_h7.c`