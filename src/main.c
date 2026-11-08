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

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_timestamp_t;

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
    sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(rtc_timestamp_t));
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

static esp_err_t init_ds1307_i2c(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100kHz standard mode
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static bool bcd_to_decimal(uint8_t bcd, uint8_t *value)
{
    uint8_t high = (bcd >> 4) & 0x0F;
    uint8_t low = bcd & 0x0F;
    if (high > 9 || low > 9)
    {
        return false;
    }

    *value = high * 10 + low;
    return true;
}

static esp_err_t read_ds1307_timestamp(rtc_timestamp_t *timestamp)
{
    uint8_t reg_addr = 0x00;
    uint8_t data[7];

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, DS1307_ADDR,
                                                 &reg_addr, 1,
                                                 data, sizeof(data),
                                                 pdMS_TO_TICKS(1000));
    xSemaphoreGive(i2c_mutex);

    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint8_t year;

    if ((data[0] & 0x80) != 0 ||
        !bcd_to_decimal(data[0] & 0x7F, &second) ||
        !bcd_to_decimal(data[1] & 0x7F, &minute) ||
        !bcd_to_decimal(data[4] & 0x3F, &day) ||
        !bcd_to_decimal(data[5] & 0x1F, &month) ||
        !bcd_to_decimal(data[6], &year))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if ((data[2] & 0x40) != 0)
    {
        uint8_t hour_12;
        if (!bcd_to_decimal(data[2] & 0x1F, &hour_12) ||
            hour_12 < 1 || hour_12 > 12)
        {
            return ESP_ERR_INVALID_RESPONSE;
        }
        hour = hour_12 % 12 + ((data[2] & 0x20) != 0 ? 12 : 0);
    }
    else if (!bcd_to_decimal(data[2] & 0x3F, &hour))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (second > 59 || minute > 59 || hour > 23 ||
        day < 1 || day > 31 || month < 1 || month > 12)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    timestamp->year = 2000 + year;
    timestamp->month = month;
    timestamp->day = day;
    timestamp->hour = hour;
    timestamp->minute = minute;
    timestamp->second = second;
    return ESP_OK;
}

static void sensor_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        rtc_timestamp_t timestamp;
        esp_err_t err = read_ds1307_timestamp(&timestamp);
        if (err == ESP_OK)
        {
            if (xQueueSend(sensor_queue, &timestamp, pdMS_TO_TICKS(100)) != pdPASS)
            {
                printf("Sensor queue is full; timestamp dropped\n");
            }
        }
        else
        {
            printf("DS1307 read failed: %s\n", esp_err_to_name(err));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

static void manager_task(void *pvParameters)
{
    rtc_timestamp_t timestamp;

    while (1)
    {
        if (xQueueReceive(sensor_queue, &timestamp, portMAX_DELAY) == pdPASS)
        {
            printf("%04u-%02u-%02u %02u:%02u:%02u\n",
                   (unsigned int)timestamp.year,
                   (unsigned int)timestamp.month,
                   (unsigned int)timestamp.day,
                   (unsigned int)timestamp.hour,
                   (unsigned int)timestamp.minute,
                   (unsigned int)timestamp.second);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_rtos_objects());
    ESP_ERROR_CHECK(init_ds1307_i2c());
    ESP_ERROR_CHECK(xTaskCreate(sensor_task, "sensor_task", 3072, NULL, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(manager_task, "manager_task", 2048, NULL, 4, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
}
