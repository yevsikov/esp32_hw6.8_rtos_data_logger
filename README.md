# Power-Aware Logger with RTOS

Mini-project for **ESP32-S3** using **ESP-IDF v5.5.2**.
The system wakes up every second, reads sensor data, updates the OLED display, and periodically stores compact binary log records into the internal flash memory.

## Project goal

This project demonstrates:

- multitasking in FreeRTOS
- synchronization with Mutex
- queue-based communication between tasks
- power saving with **Light Sleep**
- buffered logging into internal flash
- periodic sensor sampling and display updates

## Hardware

- **ESP32-S3 N16R8**
- **BME280** — temperature, humidity, pressure sensor (SPI)
- **DS1307** — real-time clock for timestamp
- **SSD1306** — OLED display
- UART terminal for commands

## Main idea

The microcontroller wakes up every **1 second**.

On each wake cycle it:

1. reads current time from **DS1307**
2. reads temperature, humidity, and pressure from **BME280**
3. updates the **SSD1306** display
4. decides whether it is time to save a log record
5. enters **Light Sleep** again for 1 second

Logging to flash is done **less often** than display updates.
For example, the display is refreshed every second, but logging can happen every **1 minute** or every **5 minutes**, depending on the compile-time setting.

To reduce flash wear, records are first collected in a **RAM buffer**.
When the buffer becomes full, it is written to flash as one sector-sized block.

## RTOS architecture

The project uses **5 FreeRTOS tasks**:

### 1. Sensor Task
Responsible for:
- reading BME280
- reading DS1307
- building one log record
- sending it to `sensor_queue`

The Sensor Task reads the DS1307 timestamp and BME280 measurements, then
sends a complete log record to `sensor_queue`.

### 2. UART Task
Responsible for:
- receiving text commands from UART
- parsing user commands
- sending commands to `command_queue`

### 3. Display Task
Responsible for:
- receiving display snapshots from `display_queue`
- updating SSD1306

The Display Task currently shows the date, time, temperature, humidity, and
pressure on the OLED. It uses the I2C mutex while updating the display so it
does not collide with the RTC read.

### 4. Manager Task
Responsible for:
- coordinating the whole application
- receiving sensor records
- receiving UART commands
- deciding when to append a record to the RAM buffer
- forcing flash flush when the buffer is full
- entering Light Sleep for 1 second

The current Manager Task receives complete sensor records from `sensor_queue`,
prints them to UART, and sends the latest snapshot to `display_queue`. Its
logging, command-handling, and sleep responsibilities will be added in later
steps.

### 5. Flash Writer Task
Responsible for:
- writing one full RAM buffer to flash
- erasing one sector before write
- advancing write offset in the log partition
- stopping logging when the partition becomes full

## Synchronization

### Mutex
The project uses an **I2C Mutex** because two devices share the bus:

- DS1307
- SSD1306

Only one task may use the I2C bus at a time.
The BME280 in the example is connected over SPI, so it does not use this mutex.

The BME280 uses SPI with CS=GPIO 10, MOSI=GPIO 11, MISO=GPIO 13, and
SCK=GPIO 12. Its Bosch SensorAPI source is included in `components/bme280`
under the BSD-3-Clause license (upstream commit
`c90d419492e26dd95586598a794e65eb2760753a`). The SPI adapter connects that
API to ESP-IDF; the SensorAPI handles calibration and measurement compensation.

### Queues
The project uses the following queues:

- `sensor_queue` — Sensor Task → Manager Task
- `command_queue` — UART Task → Manager Task
- `display_queue` — Manager Task → Display Task

The sensor queue holds 2 complete log records, the command
queue holds 5 commands, and the length-1 display queue holds the latest
snapshot.
The packed log record is 13 bytes; temperature and humidity are stored
multiplied by 100.
The queues and mutex are created during startup. Queue messages are copied by
FreeRTOS, so each message type is a value structure rather than a pointer to a
task's temporary data.

## Logging format

To save memory, logs are stored in **binary format**, not as text.

Each log record contains:

- year
- month
- day
- hour
- minute
- second
- temperature in **°C × 100**
- humidity in **% × 100**
- pressure in **mmHg**

Example:

- `23.45 °C` → `2345`
- `56.78 %` → `5678`

This is more compact than storing strings and is easier to write to flash efficiently.

## RAM buffer strategy

The project uses:

- **one RAM buffer**
- buffer size = **one flash sector**
- records are appended until the buffer is full
- when full, the whole buffer is written to flash

This reduces the number of flash write/erase operations.

## Flash logging policy

Logs are written into a dedicated internal flash partition called:

`logdata`

Important design decisions:

- **no wear leveling**
- **no metadata stored in flash**
- records are written sector by sector
- when the partition becomes full:
  - logging stops
  - warning is printed through UART
  - warning is shown on the OLED display

The checked-in `partitions.csv` targets an **8 MB** flash module. It reserves
2 MB for the application and assigns the remaining `0x5F0000` bytes to
`logdata`. Verify the module's actual flash size before flashing this layout.

## Sleep behavior

The system uses **Light Sleep** with timer wakeup.

Behavior:

- wake up every **1 second**
- perform one measurement/display cycle
- go back to Light Sleep

This demonstrates basic energy-saving behavior while still keeping the system responsive.

## UART commands

Planned commands:

- `status` — show current system status
- `flush` — force write current RAM buffer to flash
- `log on` — enable logging
- `log off` — disable logging
