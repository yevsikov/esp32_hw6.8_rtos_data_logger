#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "bme280.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define I2C_SCL 9
#define I2C_SDA 8
#define I2C_MASTER_NUM I2C_NUM_0
#define DS1307_ADDR 0x68 // DS1307 I2C address

#define BME280_MOSI 11
#define BME280_MISO 13
#define BME280_SCK 12
#define BME280_CS 10
#define BME280_SPI_HOST SPI2_HOST

#define SENSOR_QUEUE_LENGTH 2
#define COMMAND_QUEUE_LENGTH 5
#define DISPLAY_QUEUE_LENGTH 1
#define BME280_SPI_MAX_TRANSFER (BME280_LEN_TEMP_PRESS_CALIB_DATA + 1)

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
static spi_device_handle_t bme280_spi_device;
static struct bme280_dev bme280_device;
static struct bme280_settings bme280_settings;

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

static int8_t bme280_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_device_handle_t device = *(spi_device_handle_t *)intf_ptr;
    uint8_t tx_buffer[BME280_SPI_MAX_TRANSFER] = {0};
    uint8_t rx_buffer[BME280_SPI_MAX_TRANSFER] = {0};

    if (len == 0 || len > BME280_LEN_TEMP_PRESS_CALIB_DATA)
    {
        return BME280_E_INVALID_LEN;
    }

    tx_buffer[0] = reg_addr | 0x80;
    spi_transaction_t transaction = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
    };

    esp_err_t err = spi_device_transmit(device, &transaction);
    if (err != ESP_OK)
    {
        return BME280_E_COMM_FAIL;
    }

    for (uint32_t i = 0; i < len; ++i)
    {
        reg_data[i] = rx_buffer[i + 1];
    }

    return BME280_INTF_RET_SUCCESS;
}

static int8_t bme280_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_device_handle_t device = *(spi_device_handle_t *)intf_ptr;
    uint8_t tx_buffer[BME280_SPI_MAX_TRANSFER];

    if (len == 0 || len > BME280_MAX_LEN)
    {
        return BME280_E_INVALID_LEN;
    }

    tx_buffer[0] = reg_addr & 0x7F;
    for (uint32_t i = 0; i < len; ++i)
    {
        tx_buffer[i + 1] = reg_data[i];
    }

    spi_transaction_t transaction = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buffer,
    };

    return spi_device_transmit(device, &transaction) == ESP_OK
               ? BME280_INTF_RET_SUCCESS
               : BME280_E_COMM_FAIL;
}

static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}

static esp_err_t init_bme280(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = BME280_MOSI,
        .miso_io_num = BME280_MISO,
        .sclk_io_num = BME280_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BME280_SPI_MAX_TRANSFER,
    };

    esp_err_t err = spi_bus_initialize(BME280_SPI_HOST, &bus_config, SPI_DMA_DISABLED);
    if (err != ESP_OK)
    {
        return err;
    }

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = BME280_CS,
        .queue_size = 1,
    };
    err = spi_bus_add_device(BME280_SPI_HOST, &device_config, &bme280_spi_device);
    if (err != ESP_OK)
    {
        return err;
    }

    bme280_device.intf = BME280_SPI_INTF;
    bme280_device.intf_ptr = &bme280_spi_device;
    bme280_device.read = bme280_spi_read;
    bme280_device.write = bme280_spi_write;
    bme280_device.delay_us = bme280_delay_us;

    int8_t api_result = bme280_init(&bme280_device);
    if (api_result != BME280_OK)
    {
        printf("BME280 initialization failed: %d\n", api_result);
        return ESP_FAIL;
    }

    bme280_settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280_settings.osr_p = BME280_OVERSAMPLING_1X;
    bme280_settings.osr_t = BME280_OVERSAMPLING_1X;
    bme280_settings.filter = BME280_FILTER_COEFF_OFF;

    api_result = bme280_set_sensor_settings(BME280_SEL_OSR_HUM |
                                                BME280_SEL_OSR_PRESS |
                                                BME280_SEL_OSR_TEMP |
                                                BME280_SEL_FILTER,
                                            &bme280_settings, &bme280_device);
    if (api_result != BME280_OK)
    {
        printf("BME280 configuration failed: %d\n", api_result);
        return ESP_FAIL;
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

static esp_err_t read_bme280_measurements(log_record_t *record)
{
    uint32_t measurement_delay_us;
    int8_t api_result = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme280_device);
    if (api_result != BME280_OK)
    {
        printf("BME280 measurement start failed: %d\n", api_result);
        return ESP_FAIL;
    }

    bme280_cal_meas_delay(&measurement_delay_us, &bme280_settings);
    bme280_delay_us(measurement_delay_us, NULL);

    struct bme280_data data;
    api_result = bme280_get_sensor_data(BME280_ALL, &data, &bme280_device);
    if (api_result != BME280_OK)
    {
        printf("BME280 read failed: %d\n", api_result);
        return ESP_FAIL;
    }

    if (data.temperature < -40.0 || data.temperature > 85.0 ||
        data.humidity < 0.0 || data.humidity > 100.0 ||
        data.pressure < 30000.0 || data.pressure > 110000.0)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    double temperature_x100 = data.temperature * 100.0;
    record->temperature_c_x100 = (int16_t)(temperature_x100 +
                                           (temperature_x100 >= 0.0 ? 0.5 : -0.5));
    record->humidity_pct_x100 = (uint16_t)(data.humidity * 100.0 + 0.5);
    record->pressure_mmhg = (uint16_t)(data.pressure / 133.322387415);
    return ESP_OK;
}

static void sensor_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        rtc_timestamp_t timestamp;
        log_record_t record = {0};
        esp_err_t rtc_err = read_ds1307_timestamp(&timestamp);
        esp_err_t sensor_err = read_bme280_measurements(&record);

        if (rtc_err == ESP_OK && sensor_err == ESP_OK)
        {
            record.year = timestamp.year;
            record.month = timestamp.month;
            record.day = timestamp.day;
            record.hour = timestamp.hour;
            record.minute = timestamp.minute;
            record.second = timestamp.second;

            if (xQueueSend(sensor_queue, &record, pdMS_TO_TICKS(100)) != pdPASS)
            {
                printf("Sensor queue is full; measurement dropped\n");
            }
        }
        else
        {
            if (rtc_err != ESP_OK)
            {
                printf("DS1307 read failed: %s\n", esp_err_to_name(rtc_err));
            }
            if (sensor_err != ESP_OK && sensor_err != ESP_FAIL)
            {
                printf("BME280 measurement invalid: %s\n", esp_err_to_name(sensor_err));
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

static void manager_task(void *pvParameters)
{
    log_record_t record;

    while (1)
    {
        if (xQueueReceive(sensor_queue, &record, portMAX_DELAY) == pdPASS)
        {
            int32_t temperature_x100 = record.temperature_c_x100;
            uint16_t temperature_magnitude = (uint16_t)(temperature_x100 < 0
                                                            ? -temperature_x100
                                                            : temperature_x100);
            printf("%04u-%02u-%02u %02u:%02u:%02u | T=%s%u.%02u C, H=%u.%02u %%, P=%u mmHg\n",
                   (unsigned int)record.year,
                   (unsigned int)record.month,
                   (unsigned int)record.day,
                   (unsigned int)record.hour,
                   (unsigned int)record.minute,
                   (unsigned int)record.second,
                   temperature_x100 < 0 ? "-" : "",
                   (unsigned int)(temperature_magnitude / 100),
                   (unsigned int)(temperature_magnitude % 100),
                   (unsigned int)(record.humidity_pct_x100 / 100),
                   (unsigned int)(record.humidity_pct_x100 % 100),
                   (unsigned int)record.pressure_mmhg);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_rtos_objects());
    ESP_ERROR_CHECK(init_ds1307_i2c());
    ESP_ERROR_CHECK(init_bme280());
    ESP_ERROR_CHECK(xTaskCreate(sensor_task, "sensor_task", 3072, NULL, 5, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(manager_task, "manager_task", 2048, NULL, 4, NULL) == pdPASS
                        ? ESP_OK
                        : ESP_ERR_NO_MEM);
}
