/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "ws2812.pio.h"

#define NUM_LEDS 60
#define IR_THRESHOLD 0x80
#define NUM_SAMPLES 0x40000

static const int PIN_TX = 0;

static inline void put_pixel(uint32_t pixel_grb) {
  pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  return
    ((uint32_t) (r) << 8) |
    ((uint32_t) (g) << 16) |
    (uint32_t) (b);
}

int main() {
  stdio_init_all();
  adc_init();
  adc_gpio_init(26);
  adc_select_input(0);

  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, PIN_TX, 800000, false);

  int interrupts;
  uint8_t state;
  bool is_on;
  double score;
  uint8_t val;

  while (1) {
    interrupts = 0;
    state = 0;
    for (int i = 0; i < NUM_SAMPLES; ++i) {
      is_on = adc_read() > IR_THRESHOLD;
      if (state == 0 && is_on) {
        state = 1;
      }
      if (state == 1 && !is_on) {
        state = 0;
        interrupts += 1;
      }
    }
    printf("Interrupts: %d\n", interrupts);
    score = interrupts / 25.0;
    score = score > 1.0 ? 1.0 : score;
    val = 255 * score;
    for (int i = 0; i < NUM_LEDS; ++i) {
      put_pixel(urgb_u32(val / 4, val, val / 2));
    }
  }
}