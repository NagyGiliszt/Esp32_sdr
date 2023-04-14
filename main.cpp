
#include "esp32-hal.h"
#include "driver/i2s.h"
#include <Arduino.h>

#define SAMPLE_RATE (44100)
#define I2S_PORT I2S_NUM_0

void setup() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 1024,
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = 26,
    .ws_io_num = 25,
    .data_out_num = -1,
    .data_in_num = 34,
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void loop() {
  int16_t data[1024];
  size_t bytes_read;
  i2s_read(I2S_PORT, &data, sizeof(data), &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(data[0]);
  // itt folytatható a mintavételezett adatok feldolgozása
}
