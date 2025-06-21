#include <pico/multicore.h>
#include <hardware/gpio.h>

#include "graphics.h"

#define DATA0_PIN 16
#define DATA1_PIN 17
#define DATA2_PIN 18
#define DATA3_PIN 19

#define PIXEL_CLOCK_PIN 20
#define FRAME_POLARITY_PIN 21

__aligned(4) uint8_t FRAME_BUFFER[320 * 240] = {0};

void __time_critical_func(render_core)() {
    graphics_init();
    graphics_set_buffer(FRAME_BUFFER, 160, 160);
    graphics_set_textbuffer(FRAME_BUFFER);
    graphics_set_bgcolor(0x000000);
    graphics_set_offset(0, 0);

    graphics_set_palette(0, RGB888(0x7b, 0xc7, 0x7b));
    graphics_set_palette(1, RGB888(0x52, 0xa6, 0x8c));
    graphics_set_palette(2, RGB888(0x2e, 0x62, 0x60));
    graphics_set_palette(3, RGB888(0x0d, 0x32, 0x2e));

    graphics_set_flashmode(true, true);

    graphics_set_mode(GRAPHICSMODE_DEFAULT);
    // Timing variables
    uint64_t tick = time_us_64();
    uint64_t last_frame_tick = tick;
    while (true) {
        // Video frame rendering (~60Hz)
        if (tick >= last_frame_tick + 16667) {
#if defined(TFT)
            refresh_lcd();
#endif
            last_frame_tick = tick;
        }
        tight_loop_contents();
    }
}

[[noreturn]] void main() {
    // Weird, but overclocking breaks all
    // hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    // busy_wait_us(33);
    // set_sys_clock_khz(150 * 1000, true);

    gpio_init(DATA0_PIN);
    gpio_init(DATA1_PIN);
    gpio_init(DATA2_PIN);
    gpio_init(DATA3_PIN);
    gpio_init(PIXEL_CLOCK_PIN);
    gpio_init(FRAME_POLARITY_PIN);

    gpio_set_dir(DATA0_PIN, GPIO_IN);
    gpio_set_dir(DATA1_PIN, GPIO_IN);
    gpio_set_dir(DATA2_PIN, GPIO_IN);
    gpio_set_dir(DATA3_PIN, GPIO_IN);
    gpio_set_dir(PIXEL_CLOCK_PIN, GPIO_IN);
    gpio_set_dir(FRAME_POLARITY_PIN, GPIO_IN);

    gpio_pull_down(DATA0_PIN);
    gpio_pull_down(DATA1_PIN);
    gpio_pull_down(DATA2_PIN);
    gpio_pull_down(DATA3_PIN);
    gpio_pull_down(PIXEL_CLOCK_PIN);
    gpio_pull_down(FRAME_POLARITY_PIN);


    sleep_ms(50);
    multicore_launch_core1(render_core);
    sleep_ms(50);

#ifdef DEBUG
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    uint8_t led = 0;
#endif

    uint8_t last_clock_state = 0;
    uint8_t last_polarity_state = 0;

    uint8_t *pixels = FRAME_BUFFER;


    while (!gpio_get(FRAME_POLARITY_PIN)); // Wait for a new frame
    uint32_t time = time_us_32();

    while (true) {
        const uint32_t bus = gpio_get_all();
        const uint8_t clock = bus >> PIXEL_CLOCK_PIN & 1;
        const uint8_t polarity = bus >> FRAME_POLARITY_PIN & 1;

        if (clock && !last_clock_state) {
            if (polarity) {
                *pixels++ = (bus >> DATA0_PIN & 1) << 1;
                *pixels++ = (bus >> DATA1_PIN & 1) << 1;
                *pixels++ = (bus >> DATA2_PIN & 1) << 1;
                *pixels++ = (bus >> DATA3_PIN & 1) << 1;
            } else {
                *pixels++ |= bus >> DATA0_PIN & 1;
                *pixels++ |= bus >> DATA1_PIN & 1;
                *pixels++ |= bus >> DATA2_PIN & 1;
                *pixels++ |= bus >> DATA3_PIN & 1;
            }
        }

        if (polarity != last_polarity_state && time_us_32() - time > 9000) {
            pixels = FRAME_BUFFER;
            time = time_us_32();
#ifdef DEBUG
                if (polarity)
                    gpio_put(PICO_DEFAULT_LED_PIN, led ^= 1);
#endif
        }

        last_clock_state = clock;
        last_polarity_state = polarity;
    }
}
