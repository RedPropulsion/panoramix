#include "ws2812_bitbang.h"
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <stm32h7xx_ll_gpio.h> 
/* ------------------------------------------------------------------ *
 * DWT cycle-counter (Cortex-M7)
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

/* ------------------------------------------------------------------ *
 * Timing — verify with printk("CPU: %d\n", SystemCoreClock/1000000)
 * ------------------------------------------------------------------ */
#define CPU_MHZ         550U
#define NS_TO_CYCLES(ns) ((CPU_MHZ * (ns)) / 1000U)

#define T0H_CYCLES  NS_TO_CYCLES(400)
#define T0L_CYCLES  NS_TO_CYCLES(850)
#define T1H_CYCLES  NS_TO_CYCLES(800)
#define T1L_CYCLES  NS_TO_CYCLES(450)
#define RESET_US    60U

/* ------------------------------------------------------------------ *
 * Direct register bit-bang — bypasses GPIO abstraction entirely.
 * gpio_pin_set_dt() is ~300ns overhead; BSRR write is 1 cycle.
 * ------------------------------------------------------------------ */
static void send_byte(GPIO_TypeDef *port, uint32_t pin_mask, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if (byte & BIT(i)) {
            LL_GPIO_SetOutputPin(port, pin_mask);
            delay_cycles(T1H_CYCLES);
            LL_GPIO_ResetOutputPin(port, pin_mask);
            delay_cycles(T1L_CYCLES);
        } else {
            LL_GPIO_SetOutputPin(port, pin_mask);
            delay_cycles(T0H_CYCLES);
            LL_GPIO_ResetOutputPin(port, pin_mask);
            delay_cycles(T0L_CYCLES);
        }
    }
}

/* ------------------------------------------------------------------ *
 * Public API
 * ------------------------------------------------------------------ */
void ws2812_bb_init(const struct gpio_dt_spec *din_pin)
{
    dwt_enable();
    gpio_pin_configure_dt(din_pin, GPIO_OUTPUT_INACTIVE);
}

int ws2812_bb_update(const struct gpio_dt_spec *din_pin,
                     struct led_rgb *pixels, size_t num_pixels)
{
    /* Hardcode GPIOE + pin 6 — we know the hardware */
    GPIO_TypeDef *port     = GPIOE;
    uint32_t      pin_mask = LL_GPIO_PIN_6;

    unsigned int key = irq_lock();

    for (size_t i = 0; i < num_pixels; i++) {
        /* WS2812B expects GRB order */
        send_byte(port, pin_mask, pixels[i].g);
        send_byte(port, pin_mask, pixels[i].r);
        send_byte(port, pin_mask, pixels[i].b);
    }

    irq_unlock(key);

    /* Reset pulse */
    LL_GPIO_ResetOutputPin(port, pin_mask);
    k_busy_wait(RESET_US);

    return 0;
}