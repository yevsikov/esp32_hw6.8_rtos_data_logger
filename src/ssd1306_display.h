#pragma once

#include "esp_err.h"

#define SSD1306_DISPLAY_LINE_COUNT 5
#define SSD1306_DISPLAY_LINE_LENGTH 22

esp_err_t ssd1306_display_init(void);
esp_err_t ssd1306_display_write_lines(const char lines[SSD1306_DISPLAY_LINE_COUNT][SSD1306_DISPLAY_LINE_LENGTH]);
