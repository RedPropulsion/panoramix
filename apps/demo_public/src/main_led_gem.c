#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <soc.h>

/* Recupera i dati dall'overlay */
#define STRIP_NODE DT_NODELABEL(led_strip)
#define NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct device *const ws_uart = DEVICE_DT_GET(DT_NODELABEL(uart4));
static struct led_rgb pixels[NUM_PIXELS];

typedef enum ws_select {
    WS_MCU = 0,
    WS_RING
} ws_select_t;

int ws_strip_swap(ws_select_t* current_strip){
    USART_TypeDef *ws_uart_reg = (USART_TypeDef *)DT_REG_ADDR(DT_NODELABEL(uart4));


    if(!device_is_ready(ws_uart)){
        printk("Errore: UART non pronta.\n");
        return -ENODEV;
    }

    int ret = uart_tx_abort(ws_uart);
    if (ret) {
        printk("Errore: niente da abortire nella trasmissione UART. %d, \n", ret);
        //return ret;
    }

    uint32_t cr1 = ws_uart_reg->CR1;
    ws_uart_reg->CR1 &= ~USART_CR1_UE; // disable usart, necessary to change SWAP bit

    if(*current_strip == WS_MCU){
        ws_uart_reg->CR2 |= USART_CR2_SWAP; // enable swap: TX on PB8
        *current_strip = WS_RING;
    }
    else{
        ws_uart_reg->CR2 &= ~USART_CR2_SWAP; // disable swap: TX on PB9
        *current_strip = WS_MCU;
    }

    ws_uart_reg->CR1 = cr1; // restore CR1 with UE bit set to re-enable usart

    return 0;
}

static inline uint8_t rand8(void)
{
    return (uint8_t)sys_rand32_get();
}

static struct led_rgb random_color(void)
{
    return (struct led_rgb){
        .r = rand8(),
        .g = rand8(),
        .b = rand8(),
    };
}

static struct led_rgb wheel_color(uint8_t pos)
{
    if (pos < 85) {
        return (struct led_rgb){.r = (uint8_t)(255 - pos * 3), .g = (uint8_t)(pos * 3), .b = 0};
    } else if (pos < 170) {
        pos -= 85;
        return (struct led_rgb){.r = 0, .g = (uint8_t)(255 - pos * 3), .b = (uint8_t)(pos * 3)};
    }

    pos -= 170;
    return (struct led_rgb){.r = (uint8_t)(pos * 3), .g = 0, .b = (uint8_t)(255 - pos * 3)};
}

static void clear_pixels(void)
{
    memset(pixels, 0, sizeof(pixels));
}

static int show_pixels(void)
{
    return led_strip_update_rgb(strip, pixels, NUM_PIXELS);
}

static void game_single_dot(const struct led_rgb *color)
{
    int pos = 0;

    for (int step = 0; step < NUM_PIXELS * 2; step++) {
        clear_pixels();
        pixels[pos] = *color;
        show_pixels();
        k_sleep(K_MSEC(30));
        pos = (pos + 1) % NUM_PIXELS;
    }
}

static void game_tail_chase(const struct led_rgb *color)
{
    const int tail_len = 5;

    for (int step = 0; step < NUM_PIXELS * 2; step++) {
        clear_pixels();

        for (int tail = 0; tail < tail_len; tail++) {
            int idx = (step - tail + NUM_PIXELS) % NUM_PIXELS;
            int scale = 255 - tail * 55;

            pixels[idx].r = (uint8_t)((color->r * scale) / 255);
            pixels[idx].g = (uint8_t)((color->g * scale) / 255);
            pixels[idx].b = (uint8_t)((color->b * scale) / 255);
        }

        show_pixels();
        k_sleep(K_MSEC(25));
    }
}

static void game_color_wipe(const struct led_rgb *color)
{
    clear_pixels();

    for (int i = 0; i < NUM_PIXELS; i++) {
        pixels[i] = *color;
        show_pixels();
        k_sleep(K_MSEC(20));
    }
}

static void game_theater_chase(const struct led_rgb *color)
{
    for (int cycle = 0; cycle < 4; cycle++) {
        for (int phase = 0; phase < 3; phase++) {
            clear_pixels();

            for (int i = phase; i < NUM_PIXELS; i += 3) {
                pixels[i] = *color;
            }

            show_pixels();
            k_sleep(K_MSEC(70));
        }
    }
}

static void game_bounce(const struct led_rgb *color)
{
    for (int repeat = 0; repeat < 2; repeat++) {
        for (int pos = 0; pos < NUM_PIXELS; pos++) {
            clear_pixels();
            pixels[pos] = *color;
            show_pixels();
            k_sleep(K_MSEC(35));
        }

        for (int pos = NUM_PIXELS - 2; pos > 0; pos--) {
            clear_pixels();
            pixels[pos] = *color;
            show_pixels();
            k_sleep(K_MSEC(35));
        }
    }
}

static void game_blink_all(const struct led_rgb *color)
{
    for (int cycle = 0; cycle < 6; cycle++) {
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels[i] = *color;
        }
        show_pixels();
        k_sleep(K_MSEC(120));

        clear_pixels();
        show_pixels();
        k_sleep(K_MSEC(80));
    }
}

static void game_alternate(const struct led_rgb *color)
{
    for (int phase = 0; phase < 4; phase++) {
        clear_pixels();

        for (int i = phase % 2; i < NUM_PIXELS; i += 2) {
            pixels[i] = *color;
        }

        show_pixels();
        k_sleep(K_MSEC(90));
    }
}

static void game_pair_run(const struct led_rgb *color)
{
    for (int pos = 0; pos < NUM_PIXELS; pos++) {
        clear_pixels();
        pixels[pos] = *color;
        pixels[(pos + 1) % NUM_PIXELS] = *color;
        show_pixels();
        k_sleep(K_MSEC(30));
    }
}

static void game_sparkle(const struct led_rgb *color)
{
    for (int step = 0; step < 30; step++) {
        clear_pixels();

        for (int j = 0; j < 5; j++) {
            int idx = sys_rand32_get() % NUM_PIXELS;
            struct led_rgb sparkle = *color;
            int fade = 100 + (sys_rand32_get() % 156);
            sparkle.r = (uint8_t)((sparkle.r * fade) / 255);
            sparkle.g = (uint8_t)((sparkle.g * fade) / 255);
            sparkle.b = (uint8_t)((sparkle.b * fade) / 255);
            pixels[idx] = sparkle;
        }

        show_pixels();
        k_sleep(K_MSEC(40));
    }
}

static void game_gradient_sweep(const struct led_rgb *color)
{
    for (int offset = 0; offset < NUM_PIXELS; offset++) {
        for (int i = 0; i < NUM_PIXELS; i++) {
            int scale = (255 * i) / (NUM_PIXELS - 1);
            int idx = (i + offset) % NUM_PIXELS;
            pixels[idx].r = (uint8_t)((color->r * scale) / 255);
            pixels[idx].g = (uint8_t)((color->g * scale) / 255);
            pixels[idx].b = (uint8_t)((color->b * scale) / 255);
        }

        show_pixels();
        k_sleep(K_MSEC(30));
    }
}

static void game_random_scatter(const struct led_rgb *color)
{
    clear_pixels();

    for (int step = 0; step < 35; step++) {
        int idx = sys_rand32_get() % NUM_PIXELS;
        pixels[idx] = *color;
        pixels[(idx + 3) % NUM_PIXELS] = *color;

        show_pixels();
        k_sleep(K_MSEC(45));
    }
}

static void game_segment_run(const struct led_rgb *color)
{
    const int segment = 6;

    for (int pos = 0; pos < NUM_PIXELS + segment; pos++) {
        clear_pixels();

        for (int i = 0; i < segment; i++) {
            pixels[(pos + i) % NUM_PIXELS] = *color;
        }

        show_pixels();
        k_sleep(K_MSEC(35));
    }
}

static void game_pulse_all(const struct led_rgb *color)
{
    for (int step = 0; step < 2; step++) {
        for (int bright = 0; bright <= 255; bright += 25) {
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i].r = (uint8_t)((color->r * bright) / 255);
                pixels[i].g = (uint8_t)((color->g * bright) / 255);
                pixels[i].b = (uint8_t)((color->b * bright) / 255);
            }
            show_pixels();
            k_sleep(K_MSEC(35));
        }

        for (int bright = 255; bright >= 0; bright -= 25) {
            for (int i = 0; i < NUM_PIXELS; i++) {
                pixels[i].r = (uint8_t)((color->r * bright) / 255);
                pixels[i].g = (uint8_t)((color->g * bright) / 255);
                pixels[i].b = (uint8_t)((color->b * bright) / 255);
            }
            show_pixels();
            k_sleep(K_MSEC(35));
        }
    }
}

static void game_wheel_cycle(const struct led_rgb *color)
{
    ARG_UNUSED(color);

    for (int offset = 0; offset < NUM_PIXELS; offset++) {
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels[i] = wheel_color((uint8_t)((i * 256 / NUM_PIXELS + offset * 8) & 0xff));
        }
        show_pixels();
        k_sleep(K_MSEC(35));
    }
}

static void game_rainbow(const struct led_rgb *color)
{
    ARG_UNUSED(color);

    for (int shift = 0; shift < 256; shift += 8) {
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels[i] = wheel_color((uint8_t)((i * 256 / NUM_PIXELS + shift) & 0xff));
        }
        show_pixels();
        k_sleep(K_MSEC(35));
    }
}

typedef void (*light_game_t)(const struct led_rgb *color);

static const light_game_t games[] = {
    game_single_dot,
    game_tail_chase,
    game_color_wipe,
    game_theater_chase,
    game_bounce,
    game_blink_all,
    game_alternate,
    game_pair_run,
    game_sparkle,
    game_gradient_sweep,
    game_random_scatter,
    game_segment_run,
    game_pulse_all,
    game_wheel_cycle,
    game_rainbow,
};

static const char *const game_names[] = {
    "Single Dot",
    "Tail Chase",
    "Color Wipe",
    "Theater Chase",
    "Bounce",
    "Blink All",
    "Alternate",
    "Pair Run",
    "Sparkle",
    "Gradient Sweep",
    "Random Scatter",
    "Segment Run",
    "Pulse All",
    "Wheel Cycle",
    "Rainbow",
};

int main(void)
{
    

    /* 1. Verifica inizializzazione */
    if (!device_is_ready(strip)) {
        printk("Errore: dispositivo LED strip non pronto.\n");
        return -ENODEV;
    }
    
    printk("Avvio sequenza di giochi luminosi per %d LED...\n", (int)NUM_PIXELS);

    ws_select_t current_strip = WS_MCU;
    if (ws_strip_swap(&current_strip) != 0) {
        printk("Errore: impossibile selezionare la strip del ring.\n");
        return -EIO;
    }

    printk("Usando strip del ring (ring) via UART.\n");

    const int game_count = ARRAY_SIZE(games);

    while (1) {
        for (int game_idx = 0; game_idx < game_count; game_idx++) {
            struct led_rgb color = random_color();
            printk("Gioco %d/%d: %s - colore R:%u G:%u B:%u\n",
                   game_idx + 1,
                   game_count,
                   game_names[game_idx],
                   color.r,
                   color.g,
                   color.b);

            games[game_idx](&color);
            k_sleep(K_MSEC(200));
        }
    }

    return 0;
}