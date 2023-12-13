#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include <stdlib.h>
#include <string.h>

#include "ssd1306.h"

#define tag "SSD1306"

#define SHT31_I2C_SCL 25
#define SHT31_I2C_SDA 26
#define SHT31_I2C I2C_NUM_1
#define SHT31_I2C_FREQ 100000 // = 100 MHz

#define SHT3X_I2C_ADDRESS 0x44

void sh31_i2c_master_init() {
    i2c_config_t i2c_config;
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = SHT31_I2C_SDA;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_io_num = SHT31_I2C_SCL;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = SHT31_I2C_FREQ;

    i2c_param_config(SHT31_I2C, &i2c_config);
    i2c_driver_install(SHT31_I2C, i2c_config.mode, 0, 0, 0);
}

void sht3x_read_data(float *temperature, float *humidity) {
    uint8_t command[] = {0x2C, 0x06}; // page 10 high repeatability measurement

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT3X_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, command, sizeof(command), true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(SHT31_I2C, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(20 / portTICK_PERIOD_MS); // Wait for measurement to complete

    // Read 6 bytes of data (temperature and humidity)
    uint8_t data[6];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT3X_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(SHT31_I2C, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Convert raw data to temperature and humidity
    int32_t raw_temperature = (data[0] << 8) | data[1];
    int32_t raw_humidity = (data[3] << 8) | data[4];

    *temperature = -45.0 + 175.0 * (float) raw_temperature / 65535.0; // doc page 13
    *humidity = 100.0 * (float) raw_humidity / 65535.0;
}

void app_main() {
    sh31_i2c_master_init();

    SSD1306_t dev;

    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&dev, 128, 64);

    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);

    bool mode = false;

    while (1) {
        float temperature, humidity;

        if (mode) {
            for (int i = 0; i < 6; i++) {
                sht3x_read_data(&temperature, &humidity);

                char temperatureString[16];
                char humidityString[16];

                // Convert float to string with two decimal places
                sprintf(temperatureString, "%.2f", temperature);

                sprintf(humidityString, "%.2f", humidity);

                printf(temperatureString);
                printf(" : ");
                printf(humidityString);
                printf("\n");

                ssd1306_display_text_x3(&dev, 3, temperatureString, 16, false);

                vTaskDelay(500 / portTICK_PERIOD_MS); // Pause for 0.5 second
            }
        } else {
            for (int i = 0; i < 6; i++) {
                sht3x_read_data(&temperature, &humidity);

                char temperatureString[16];
                char humidityString[16];

                // Convert float to string with two decimal places
                sprintf(temperatureString, "%.2f", temperature);

                sprintf(humidityString, "%.2f", humidity);

                printf(temperatureString);
                printf(" : ");
                printf(humidityString);
                printf("\n");
                ssd1306_display_text_x3(&dev, 3, humidityString, 16, false);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Pause for 0.5 second
            }
        }

        ssd1306_clear_screen(&dev, false);
        mode = !mode;
    }
}
