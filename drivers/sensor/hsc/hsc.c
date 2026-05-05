/* Honeywell HSCx Series Pressure sensors
 * For SPI interface development the Technical note [SPI Communications with Honeywell Digital Output
Pressure Sensors](https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/en-us/products/sensors/pressure-sensors/common/documents/sps-siot-spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-ciid-45843.pdf)
 * was used.
 * This library should be valid for all Honeywell sensors with SPI interface.
 * 
 * All sensors are programmed to output 4 bytes of data but not all 4 might be valid
 * You don't need to send any message to it.
 * 
 * Keep in mind that this sensor is quite  "slow" as its maximum sck frequency is of 800kHz
*/


#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT honeywell_hsc

LOG_MODULE_REGISTER(hsc, CONFIG_SENSOR_LOG_LEVEL);

struct hsc_sample
{
    int16_t pressure;
    int16_t temperature;
};

struct hsc_config
{
    struct spi_dt_spec spi;
};
