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
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

// By default these devices  are on bus address 0x68
static int addr = 0x68;
static const double G_VALUE = 9.81;
static const double sample_rate_s = 0.1;
static const uint8_t NUM_LEDS = 100;
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

static double magnitude(int16_t a, int16_t b, int16_t c) {
  return sqrt(a * a + b * b + c * c);
}

static double scaleToG(double val) {
  return val * 2 / 32767;
}

static double boundValue(double val, double limit, double alpha) {
  int outerSign = val >= 0 ? 1 : -1;
  int exponentialSign = val >= 0 ? -1 : 1;
  return outerSign * limit * (1 - exp(exponentialSign * alpha * val));
}

static void mpu6050_reset() {
  // Two byte reset. First byte register, second byte data
  // There are a load more options to set up the device in different ways that could be added here
  uint8_t buf[] = {0x6B, 0x00};
  i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  uint8_t buffer[6];

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
  i2c_read_blocking(i2c_default, addr, buffer, 6, false);

  for (int i = 0; i < 3; i++) {
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }

  // Now gyro data from reg 0x43 for 6 bytes
  // The register is auto incrementing on each read
  val = 0x43;
  i2c_write_blocking(i2c_default, addr, &val, 1, true);
  i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

  for (int i = 0; i < 3; i++) {
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
  }

  // Now temperature from reg 0x41 for 2 bytes
  // The register is auto incrementing on each read
  val = 0x41;
  i2c_write_blocking(i2c_default, addr, &val, 1, true);
  i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

  *temp = buffer[0] << 8 | buffer[1];
}

int main() {
  //set_sys_clock_48();
  stdio_init_all();
  // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  mpu6050_reset();

  int16_t acceleration[3], gyro[3], temp;

  // todo get free sm
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, PIN_TX, 800000, false);

  while (1) {
    mpu6050_read_raw(acceleration, gyro, &temp);
    double y_accel = acceleration[1];
    double y_accel_g = scaleToG(y_accel);
    printf("Magnitude [m/s]: %f\n", (y_accel_g * G_VALUE));
    double y_accel_g_limited = boundValue(y_accel_g, 2, 1);
    int shift_amount = (int)(y_accel_g_limited * 50);
    printf("Shift: %d\n", (shift_amount));


    for (int i = 0; i < NUM_LEDS / 2; ++i) {
      put_pixel(urgb_u32(100 - shift_amount, 0, 100 + shift_amount));
    }
    sleep_ms((int)(sample_rate_s * 1000));
  }
}