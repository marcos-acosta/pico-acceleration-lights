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
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

// #define MPU6050_GYRO_CONFIG 0x1b
// #define FS_SEL_250 0x00
// #define FS_SEL_500 0x08
// #define FS_SEL_1000 0x10
// #define FS_SEL_2000 0x18

#define NUM_LEDS 60
#define IR_THRESHOLD 0x80
#define NUM_SAMPLES 0x40000

// By default these devices  are on bus address 0x68
// static int addr = 0x68;
// static const double G_VALUE = 9.81;
// static const double sample_rate_s = 0.01;
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

// static double scaleGyroToDegPerSec(double val, int scaleFactor) {
//   return val * scaleFactor / 32750;
// }

// static void mpu6050_reset(uint8_t sensitivity) {
//   // Two byte reset. First byte register, second byte data
//   // There are a load more options to set up the device in different ways that could be added here
//   uint8_t buf[] = {0x6B, 0x00};
//   uint8_t sensitivity_config[] = {MPU6050_GYRO_CONFIG, sensitivity};
//   i2c_write_blocking(i2c_default, addr, buf, 2, false);
//   i2c_write_blocking(i2c_default, addr, sensitivity_config, 2, false);
// }

// static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
//   // For this particular device, we send the device the register we want to read
//   // first, then subsequently read from the device. The register is auto incrementing
//   // so we don't need to keep sending the register we want, just the first.

//   uint8_t buffer[6];

//   // Start reading acceleration registers from register 0x3B for 6 bytes
//   uint8_t val = 0x3B;
//   i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
//   i2c_read_blocking(i2c_default, addr, buffer, 6, false);

//   for (int i = 0; i < 3; i++) {
//     accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
//   }

//   // Now gyro data from reg 0x43 for 6 bytes
//   // The register is auto incrementing on each read
//   val = 0x43;
//   i2c_write_blocking(i2c_default, addr, &val, 1, true);
//   i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

//   for (int i = 0; i < 3; i++) {
//     gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
//   }

//   // Now temperature from reg 0x41 for 2 bytes
//   // The register is auto incrementing on each read
//   val = 0x41;
//   i2c_write_blocking(i2c_default, addr, &val, 1, true);
//   i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

//   *temp = buffer[0] << 8 | buffer[1];
// }

// int main_old() {
//   //set_sys_clock_48();
//   stdio_init_all();
//   // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
//   i2c_init(i2c_default, 400 * 1000);
//   gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
//   gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
//   gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
//   gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
//   // Make the I2C pins available to picotool
//   bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

//   uint8_t sensitivity = FS_SEL_2000;
//   int sensitivity_scale = 2000;
//   mpu6050_reset(sensitivity);

//   int16_t acceleration[3], gyro[3], temp;

//   // todo get free sm
//   PIO pio = pio0;
//   int sm = 0;
//   uint offset = pio_add_program(pio, &ws2812_program);

//   ws2812_program_init(pio, sm, offset, PIN_TX, 800000, false);

//   while (1) {
//     mpu6050_read_raw(acceleration, gyro, &temp);

//     double z_rotation = gyro[2];
//     int limit = 1000;
//     int max_brightness = 255;
//     printf("Raw value: %f\n", z_rotation);
//     double z_rotation_dps = scaleGyroToDegPerSec(z_rotation, sensitivity_scale);
//     printf("Deg/s: %f\n", z_rotation_dps);
//     double z_rotation_dps_bounded = fmin(z_rotation_dps, limit);
//     printf("Deg/s (bounded): %f\n", z_rotation_dps_bounded);
//     int shift_amount = fabs((int)(z_rotation_dps_bounded * max_brightness / limit));
//     printf("Shift: %d\n", shift_amount);
//     printf("\n");

//     for (int i = 0; i < NUM_LEDS; ++i) {
//       put_pixel(urgb_u32(0, shift_amount, shift_amount/2));
//     }
//     sleep_ms((int)(sample_rate_s * 1000));
//   }
// }

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
      // sleep_ms(1);
    }
    printf("Interrupts: %d\n", interrupts);
    score = interrupts / 25.0;
    score = score > 1.0 ? 1.0 : score;
    val = 255 * score;
    for (int i = 0; i < NUM_LEDS; ++i) {
      put_pixel(urgb_u32(val / 4, val, val / 2));
    }
    // sleep_ms(100);
  }
}