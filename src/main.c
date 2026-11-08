#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define I2C_SCL 9
#define I2C_SDA 8
#define I2C_MASTER_NUM I2C_NUM_0
#define DS1307_ADDR 0x68 // DS1307 I2C address

#define SENSOR_QUEUE_LENGTH 2
#define COMMAND_QUEUE_LENGTH 5
#define DISPLAY_QUEUE_LENGTH 1

typedef struct __attribute__((packed))
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    int16_t temperature_c_x100;
    uint16_t humidity_pct_x100;
    uint16_t pressure_mmhg;
} log_record_t;

_Static_assert(sizeof(log_record_t) == 13, "Unexpected log record size");

typedef enum
{
    UART_COMMAND_STATUS,
    UART_COMMAND_LOG_ON,
    UART_COMMAND_LOG_OFF,
    UART_COMMAND_FLUSH
} uart_command_type_t;

typedef struct
{
    uart_command_type_t type;
} uart_command_t;

typedef struct
{
    log_record_t record;
    bool logging_enabled;
    bool partition_full;
} display_snapshot_t;

static QueueHandle_t sensor_queue;
static QueueHandle_t command_queue;
static QueueHandle_t display_queue;
static SemaphoreHandle_t i2c_mutex;

static esp_err_t init_rtos_objects(void)
{
    sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(log_record_t));
    command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(uart_command_t));
    display_queue = xQueueCreate(DISPLAY_QUEUE_LENGTH, sizeof(display_snapshot_t));
    i2c_mutex = xSemaphoreCreateMutex();

    if (sensor_queue == NULL || command_queue == NULL ||
        display_queue == NULL || i2c_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void init_ds1307_i2c()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100kHz standard mode
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

uint8_t bcd_to_decimal(uint8_t val)
{
    return ((val / 16 * 10) + (val % 16));
}

void rtc_read_task(void *pvParameters)
{
    while (1)
    {
        uint8_t reg_addr = 0x00; // Start reading at register 0x00 (Seconds)
        uint8_t data[7];         // Buffer for sec, min, hour, day, date, month, year

        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            // I2C combined transaction: Write register address then read 7 bytes
            esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, DS1307_ADDR,
                                                         &reg_addr, 1,
                                                         data, 7,
                                                         pdMS_TO_TICKS(1000));
            xSemaphoreGive(i2c_mutex);

            if (ret == ESP_OK)
            {
                // DS1307 data format: data[0]=sec, data[1]=min, data[2]=hour
                printf("Time: %02d:%02d:%02d\n",
                       bcd_to_decimal(data[2] & 0x3F),  // Hours (bit 6 is 12/24h toggle)
                       bcd_to_decimal(data[1]),         // Minutes
                       bcd_to_decimal(data[0] & 0x7F)); // Seconds (bit 7 is Clock Halt)
            }
            else
            {
                printf("I2C Read Failed: %s\n", esp_err_to_name(ret));
            }
        }
        else
        {
            printf("I2C mutex timeout\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(init_rtos_objects());
    init_ds1307_i2c();
    xTaskCreate(rtc_read_task, "rtc_read_task", 2048, NULL, 5, NULL);
}
