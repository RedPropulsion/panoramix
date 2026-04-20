# LoRa and SX1261 - Documentation

## What is LoRa

LoRa (Long Range) is a spread spectrum wireless communication technology developed by Semtech. It is designed for long-distance communications (up to several km under ideal conditions) with low power consumption.

### Key characteristics

- **Long range**: up to 2-5 km in urban areas, 10+ km in rural areas
- **Low power**: transmissions in the milliwatt range
- **Narrow bandwidth**: typically 125 kHz or 250 kHz
- **Spreading Factor (SF)**: from 7 to 12 - higher SF = more range but slower
- **Duty cycle**: limited to 1% in Europe (868 MHz)

## Configuration Parameters

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| Frequency | Operating frequency | 868 MHz (EU), 915 MHz (US) |
| Bandwidth | Bandwidth | 125 kHz, 250 kHz, 500 kHz |
| Spreading Factor | Spreading factor | SF7-SF12 |
| Coding Rate | Coding rate | CR 4/5, CR 4/6, CR 4/7, CR 4/8 |
| Preamble | Preamble length | 8-12 symbols |
| TX Power | Transmit power | 2-22 dBm |

## SX1261 vs SX1262

The Semtech SX1261 is the low-power version of the LoRa transceiver:

| Characteristic | SX1261 | SX1262 |
|----------------|--------|--------|
| Max TX Power | 14 dBm | 22 dBm |
| TX Current | ~40 mA @ 14 dBm | ~120 mA @ 20 dBm |
| Max Packet | 255 bytes | 255 bytes |

The Obelics uses SX1261 with 4 dBm power.

## SX1261 Pins

### Control Pins

| Pin | Function | Description |
|-----|----------|-------------|
| NSS | Chip Select | SPI device select (active low) |
| SCK | SPI Clock | Serial communication clock |
| MOSI | Master Out Slave In | Data from MCU to LoRa |
| MISO | Master In Slave Out | Data from LoRa to MCU |
| RESET | Reset | Chip reset (active low) |
| BUSY | Busy | Indicates chip is busy (must be low to communicate) |
| DIO1 | Digital I/O 1 | Interrupt for TX Done, RX Done, etc. |

### RF Pins

| Pin | Function |
|-----|----------|
| ANT | Antenna |
| RFI | RF input (PA boost) |
| RFO | RF output (internal PA) |

## Obelics Configuration

### Device Tree (nucleo_h723zg.overlay)

```
&spi1 {
    status = "okay";
    pinctrl-0 = <&spi1_sck_pa5 &spi1_miso_pa6 &spi1_mosi_pb5>;
    cs-gpios = <&gpiog 12 (GPIO_ACTIVE_LOW)>;

    lora_sx1261: lora@0 {
        status = "okay";
        compatible = "semtech,sx1261";
        reg = <0>;
        spi-max-frequency = <1000000>;
        label = "SX1261";

        reset-gpios = <&gpioa 3 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        dio1-gpios = <&gpioe 11 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
        busy-gpios = <&gpioe 13 (GPIO_PULL_UP | GPIO_ACTIVE_HIGH)>;
    };
};
```

### Physical Pin Mapping

| Function | STM32 Pin | Connector |
|----------|-----------|-----------|
| SCK | PA5 | D13 (Arduino) |
| MISO | PA6 | D12 (Arduino) |
| MOSI | PB5 | D11 (Arduino) |
| NSS (CS) | PG12 | D7 (Arduino) |
| RESET | PA3 | A0 (Arduino) |
| DIO1 | PE11 | D15 (Arduino) |
| BUSY | PE13 | D14 (Arduino) |

### Configuration in main.c

```c
struct lora_modem_config config;
config.frequency = 868000000;      // 868 MHz (EU)
config.bandwidth = BW_125_KHZ;     // 125 kHz
config.datarate = SF_7;            // Spreading Factor 7
config.coding_rate = CR_4_5;       // Coding rate 4/5
config.preamble_len = 12;
config.tx_power = 4;               // 4 dBm
config.tx = true;                  // Transmit mode
config.iq_inverted = false;
config.public_network = false;

lora_config(lora_dev, &config);
```

## Important Notes

### NSS and SPI Management

The NSS (chip select) pin must be controlled correctly. In the device tree it is configured as `cs-gpios` which automatically handles chip selection.

### Busy Pin

The BUSY pin is critical: when high, the chip is executing an operation and will not accept new commands. The driver waits for BUSY to be low before sending SPI commands.

### DIO1

DIO1 is configured as an interrupt to signal events such as:
- TX Done (transmission complete)
- RX Done (data received)
- RX Timeout

### TCXO

The SX1261 can use a TCXO (Temperature Compensated Crystal Oscillator) for greater frequency stability. The SX1261MB2xAS module **does not have a TCXO**, so do not configure `dio3-tcxo-voltage` and `tcxo-power-startup-delay-ms` properties.

## References

- [Semtech SX1261 Datasheet](https://www.semtech.com/products/wireless-rf/lora-connect/sx1261)
- [Zephyr LoRa API](https://docs.zephyrproject.org/latest/drivers/lora/api.html)
- [Zephyr SX126X Driver](https://github.com/zephyrproject-rtos/zephyr/tree/main/drivers/lora/loramac_node)