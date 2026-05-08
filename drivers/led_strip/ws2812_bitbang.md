# Timing
Each LED watched the DataIN line for informations, lenght of pulse encodes symbol (`0` ir `1`).
```
'0' bit:  ▔▔╗___________   400ns high, 850ns low
'1' bit:  ▔▔▔▔▔▔╗_______   800ns high, 450ns low
```
When it sees a 50µs low from the DIN line it is told to lath/display the result.
Each LED consumes the first `8x3` bits (3 color bytes) and passes the rest to the next pixel in the strip.

# CPU Cycles
On the H7 we can't use `gpio_pin_set_dt()` because it uses a lot of CPU cycles, @ $550MHz$ that is approx 300ns per per call, the lenght of a `0` symbol high pulse. So even if you intend to send a `0` the LED will detect a `1` regardless. So you get all white LEDs all of the time.

## DWT
DWT stands for Data Watchpoint and Trace. It is a Hardware counter that increments every CPU clocl cycle, it is part of the Cortex-M7 core.
So `DWT->CYCCNT` is a very precise counter.

We just busy wait the exact amount of cycles we need using `delay_cycles`, so we don't care what else is happening.

# Brute Force 
For speed instead of using Zephyr abstractions we access the STM32 BSRR (Bit Set/ Reset Register) directly via the STM32 LL Api.

We also Disable Interrupt Requests(IRQ) for this duration to ensure precise bit signal lenght, We can do this only because we only have 2-4 Leds and zephyr is designed to being able to handle it. Shouldn't be an issue. Transmission time is ~60µs