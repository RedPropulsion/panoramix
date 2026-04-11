#define DT_DRV_COMPAT panoramix_ws2812_gpio_stm32

#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/dt-bindings/led/led.h>
#include <stm32h7xx_ll_gpio.h>

struct ws2812_stm32_config {
    GPIO_TypeDef *port;       /* resolved at compile time from DT */
    uint32_t      pin_mask;   /* resolved at compile time from DT */
    uint8_t num_colors;
    const uint8_t *color_mapping;
    size_t length;
    uint32_t cpu_mhz;
};

struct gpio_dt_spec pin =
        GPIO_DT_SPEC_GET(DT_DRV_INST(0), gpios);

/* ------------------------------------------------------------------ *
 * DWT helpers
 * ------------------------------------------------------------------ */
static inline void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_cycles(void)
{
    return DWT->CYCCNT;
}

static inline void delay_cycles(uint32_t cycles)
{
    uint32_t start = dwt_cycles();
    while ((dwt_cycles() - start) < cycles) {}
}

#define NS_TO_CYCLES(mhz, ns)  ((mhz) * (ns) / 1000U)

static void send_byte(GPIO_TypeDef *port, uint32_t pin_mask,
                      uint8_t byte, uint32_t cpu_mhz)
{
    uint32_t t0h = NS_TO_CYCLES(cpu_mhz, 400);
    uint32_t t0l = NS_TO_CYCLES(cpu_mhz, 850);
    uint32_t t1h = NS_TO_CYCLES(cpu_mhz, 800);
    uint32_t t1l = NS_TO_CYCLES(cpu_mhz, 450);

    for (int i = 7; i >= 0; i--) {
        if (byte & BIT(i)) {
            LL_GPIO_SetOutputPin(port, pin_mask);
            delay_cycles(t1h);
            LL_GPIO_ResetOutputPin(port, pin_mask);
            delay_cycles(t1l);
        } else {
            LL_GPIO_SetOutputPin(port, pin_mask);
            delay_cycles(t0h);
            LL_GPIO_ResetOutputPin(port, pin_mask);
            delay_cycles(t0l);
        }
    }
}

static int ws2812_stm32_update_rgb(const struct device *dev,
                                   struct led_rgb *pixels,
                                   size_t num_pixels)
{
    const struct ws2812_stm32_config *cfg = dev->config;

    unsigned int key = irq_lock();

    for (size_t i = 0; i < num_pixels; i++) {
        for (uint8_t c = 0; c < cfg->num_colors; c++) {
            switch (cfg->color_mapping[c]) {
            case LED_COLOR_ID_RED:
                send_byte(cfg->port, cfg->pin_mask, pixels[i].r, cfg->cpu_mhz);
                break;
            case LED_COLOR_ID_GREEN:
                send_byte(cfg->port, cfg->pin_mask, pixels[i].g, cfg->cpu_mhz);
                break;
            case LED_COLOR_ID_BLUE:
                send_byte(cfg->port, cfg->pin_mask, pixels[i].b, cfg->cpu_mhz);
                break;
            }
        }
    }

    irq_unlock(key);

    LL_GPIO_ResetOutputPin(cfg->port, cfg->pin_mask);
    k_busy_wait(60U);

    return 0;
}

static size_t ws2812_stm32_length(const struct device *dev)
{
    const struct ws2812_stm32_config *cfg = dev->config;
    return cfg->length;
}

static int ws2812_stm32_init(const struct device *dev)
{
    const struct ws2812_stm32_config *cfg = dev->config;

    if (!gpio_is_ready_dt(&pin)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&pin, GPIO_OUTPUT_INACTIVE);
    dwt_enable();
    return 0;
}

static DEVICE_API(led_strip, ws2812_stm32_api) = {
    .update_rgb = ws2812_stm32_update_rgb,
    .length     = ws2812_stm32_length,
};

/* ------------------------------------------------------------------ *
 * Port and pin resolved at compile time — no runtime DT lookup needed
 * ------------------------------------------------------------------ */
#define WS2812_STM32_COLOR_MAPPING(idx) \
    static const uint8_t ws2812_stm32_##idx##_color_mapping[] = \
        DT_INST_PROP(idx, color_mapping)

#define WS2812_STM32_DEFINE(idx)                                            \
    WS2812_STM32_COLOR_MAPPING(idx);                                        \
                                                                            \
    static const struct ws2812_stm32_config ws2812_stm32_##idx##_cfg = {   \
        /* ← get port base address and pin directly from DT instance */     \
        .port      = (GPIO_TypeDef *)DT_REG_ADDR(                           \
                        DT_PHANDLE(DT_DRV_INST(idx), gpios)),               \
        .pin_mask  = BIT(DT_INST_GPIO_PIN(idx, gpios)),                     \
        .num_colors    = DT_INST_PROP_LEN(idx, color_mapping),              \
        .color_mapping = ws2812_stm32_##idx##_color_mapping,                \
        .length        = DT_INST_PROP(idx, chain_length),                   \
        .cpu_mhz       = DT_INST_PROP(idx, cpu_mhz),                       \
    };                                                                      \
                                                                            \
    DEVICE_DT_INST_DEFINE(idx,                                              \
        ws2812_stm32_init, NULL, NULL,                                      \
        &ws2812_stm32_##idx##_cfg,                                          \
        POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY,                        \
        &ws2812_stm32_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_STM32_DEFINE)