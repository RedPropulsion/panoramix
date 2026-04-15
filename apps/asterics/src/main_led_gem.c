#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
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
    

int main(void)
{
    

    /* 1. Verifica inizializzazione */
    if (!device_is_ready(strip)) {
        printk("Errore: dispositivo LED strip non pronto.\n");
        return -ENODEV;
    }
    
    printk("Avvio test per %d LED...\n", (int)NUM_PIXELS);
    
    /* 3 Colori primari */
    struct led_rgb colors[] = {
        { .r = 0xff, .g = 0x00, .b = 0x00 }, /* Rosso */
        { .r = 0x00, .g = 0xff, .b = 0x00 }, /* Verde */
        { .r = 0x00, .g = 0x00, .b = 0xff }, /* Blu   */
    };

    int color_idx = 0;
    ws_select_t* current_strip = WS_MCU;
    
    while (1) {
        for (int i = 0; i < NUM_PIXELS; i++) {
            
            /* Print per monitorare esattamente dove siamo */
            printk("Accendo LED %d con colore indice %d...\n", i, color_idx);
            
            /* Spegne tutti e accende solo il corrente */
            memset(pixels, 0, sizeof(pixels));
            pixels[i] = colors[color_idx];

            /* Invia i dati (Qui è dove prima si bloccava!) */
            int rc = led_strip_update_rgb(strip, pixels, NUM_PIXELS);
            if (rc) {
                printk("Errore durante l'aggiornamento dei LED: %d\n", rc);
            }

            k_sleep(K_MSEC(100));
        }
        
        color_idx = (color_idx + 1) % ARRAY_SIZE(colors);
        ws_strip_swap(current_strip);
    }

    return 0;
}