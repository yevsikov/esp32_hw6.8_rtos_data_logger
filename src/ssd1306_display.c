#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "ssd1306_display.h"

#define DISPLAY_I2C_PORT I2C_NUM_0
#define DISPLAY_I2C_ADDRESS 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_PAGE_COUNT (DISPLAY_HEIGHT / 8)
#define DISPLAY_TRANSFER_CHUNK 16
#define DISPLAY_I2C_TIMEOUT_MS 1000

static uint8_t framebuffer[DISPLAY_WIDTH * DISPLAY_PAGE_COUNT];

static esp_err_t write_command(uint8_t command)
{
    uint8_t data[] = {0x00, command};
    return i2c_master_write_to_device(DISPLAY_I2C_PORT, DISPLAY_I2C_ADDRESS,
                                      data, sizeof(data),
                                      pdMS_TO_TICKS(DISPLAY_I2C_TIMEOUT_MS));
}

static esp_err_t write_commands(const uint8_t *commands, size_t length)
{
    uint8_t data[sizeof(uint8_t) + 32];
    if (length > sizeof(data) - 1)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    data[0] = 0x00;
    memcpy(&data[1], commands, length);
    return i2c_master_write_to_device(DISPLAY_I2C_PORT, DISPLAY_I2C_ADDRESS,
                                      data, length + 1,
                                      pdMS_TO_TICKS(DISPLAY_I2C_TIMEOUT_MS));
}

static void get_glyph(char character, uint8_t glyph[5])
{
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E},
    };

    memset(glyph, 0, 5);
    if (character >= '0' && character <= '9')
    {
        memcpy(glyph, digits[character - '0'], 5);
        return;
    }

    switch (character)
    {
    case '-':
        glyph[1] = 0x08; glyph[2] = 0x08; glyph[3] = 0x08;
        break;
    case '.':
        glyph[2] = 0x60;
        break;
    case ':':
        glyph[2] = 0x36;
        break;
    case '%':
        glyph[0] = 0x63; glyph[1] = 0x13; glyph[2] = 0x08;
        glyph[3] = 0x64; glyph[4] = 0x63;
        break;
    case 'T':
        glyph[0] = 0x01; glyph[1] = 0x01; glyph[2] = 0x7F;
        glyph[3] = 0x01; glyph[4] = 0x01;
        break;
    case 'H':
        glyph[0] = 0x7F; glyph[1] = 0x08; glyph[2] = 0x08;
        glyph[3] = 0x08; glyph[4] = 0x7F;
        break;
    case 'P':
        glyph[0] = 0x7F; glyph[1] = 0x09; glyph[2] = 0x09;
        glyph[3] = 0x09; glyph[4] = 0x06;
        break;
    case 'C':
        glyph[0] = 0x3E; glyph[1] = 0x41; glyph[2] = 0x41;
        glyph[3] = 0x41; glyph[4] = 0x22;
        break;
    case 'm':
        glyph[0] = 0x7C; glyph[1] = 0x04; glyph[2] = 0x18;
        glyph[3] = 0x04; glyph[4] = 0x78;
        break;
    case 'g':
        glyph[0] = 0x0C; glyph[1] = 0x52; glyph[2] = 0x52;
        glyph[3] = 0x52; glyph[4] = 0x3E;
        break;
    default:
        break;
    }
}

static void draw_text(uint8_t x, uint8_t y, const char *text)
{
    while (*text != '\0' && x + 5 < DISPLAY_WIDTH)
    {
        uint8_t glyph[5];
        get_glyph(*text++, glyph);

        for (uint8_t column = 0; column < 5; ++column)
        {
            for (uint8_t row = 0; row < 7; ++row)
            {
                uint8_t pixel_y = y + row;
                if ((glyph[column] & (1U << row)) != 0)
                {
                    uint16_t pixel = (uint16_t)(pixel_y / 8) * DISPLAY_WIDTH + x + column;
                    framebuffer[pixel] |= 1U << (pixel_y % 8);
                }
            }
        }

        x += 6;
    }
}

static esp_err_t write_framebuffer(void)
{
    esp_err_t err;
    for (uint8_t page = 0; page < DISPLAY_PAGE_COUNT; ++page)
    {
        err = write_command(0xB0 | page);
        if (err != ESP_OK)
        {
            return err;
        }
        err = write_command(0x00);
        if (err != ESP_OK)
        {
            return err;
        }
        err = write_command(0x10);
        if (err != ESP_OK)
        {
            return err;
        }

        for (uint8_t column = 0; column < DISPLAY_WIDTH; column += DISPLAY_TRANSFER_CHUNK)
        {
            uint8_t data[DISPLAY_TRANSFER_CHUNK + 1];
            data[0] = 0x40;
            for (uint8_t i = 0; i < DISPLAY_TRANSFER_CHUNK; ++i)
            {
                data[i + 1] = framebuffer[(uint16_t)page * DISPLAY_WIDTH + column + i];
            }

            err = i2c_master_write_to_device(DISPLAY_I2C_PORT, DISPLAY_I2C_ADDRESS,
                                             data, sizeof(data),
                                             pdMS_TO_TICKS(DISPLAY_I2C_TIMEOUT_MS));
            if (err != ESP_OK)
            {
                return err;
            }
        }
    }

    return ESP_OK;
}

esp_err_t ssd1306_display_init(void)
{
    static const uint8_t init_commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
    };
    esp_err_t err = write_commands(init_commands, sizeof(init_commands));
    if (err != ESP_OK)
    {
        return err;
    }

    memset(framebuffer, 0, sizeof(framebuffer));
    return write_framebuffer();
}

esp_err_t ssd1306_display_write_lines(const char lines[SSD1306_DISPLAY_LINE_COUNT][SSD1306_DISPLAY_LINE_LENGTH])
{
    memset(framebuffer, 0, sizeof(framebuffer));
    for (uint8_t line = 0; line < SSD1306_DISPLAY_LINE_COUNT; ++line)
    {
        draw_text(0, line * 12, lines[line]);
    }

    return write_framebuffer();
}
