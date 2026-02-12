/*
 * ssd1306.h - SSD1306 128x64 OLED driver over I2C
 *
 * Minimal framebuffer-based driver. We keep a 128x64 bit
 * buffer in RAM (1024 bytes) and flush the whole thing on
 * each update. For a 100kHz I2C bus this takes ~100ms which
 * is fine for a diagnostic display refreshing a few times
 * per second.
 *
 * Font is a built-in 6x8 fixed-width ASCII subset.
 */

#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SSD1306_ADDR        (0x3C << 1)
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64
#define SSD1306_BUF_SIZE    (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

typedef enum {
    SSD1306_OK = 0,
    SSD1306_ERR_I2C,
} ssd1306_status_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            buf[SSD1306_BUF_SIZE];
} ssd1306_dev_t;

ssd1306_status_t ssd1306_init(ssd1306_dev_t *dev, I2C_HandleTypeDef *hi2c);
void             ssd1306_clear(ssd1306_dev_t *dev);
void             ssd1306_pixel(ssd1306_dev_t *dev, uint8_t x, uint8_t y, bool on);
void             ssd1306_putchar(ssd1306_dev_t *dev, uint8_t x, uint8_t y, char c);
void             ssd1306_puts(ssd1306_dev_t *dev, uint8_t x, uint8_t y, const char *s);
ssd1306_status_t ssd1306_flush(ssd1306_dev_t *dev);

#endif /* SSD1306_H */
