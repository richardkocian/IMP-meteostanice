#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

#include "ssd1306.h"

#define tag "SSD1306"

#define I2C_MASTER_SCL_IO 25
#define I2C_MASTER_SDA_IO 26
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000

#define SHT3X_I2C_ADDRESS 0x44

void sh31_i2c_master_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, 0, 0, 0);
}

void sht3x_read_data(float *temperature, float *humidity) {
    uint8_t command[] = {0x2C, 0x06}; // High precision measurement command

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT3X_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, command, sizeof(command), true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(20 / portTICK_PERIOD_MS); // Wait for measurement to complete

    // Read 6 bytes of data (temperature and humidity)
    uint8_t data[6];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT3X_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Convert raw data to temperature and humidity
    int32_t raw_temperature = (data[0] << 8) | data[1];
    int32_t raw_humidity = (data[3] << 8) | data[4];

    *temperature = -45.0 + 175.0 * (float)raw_temperature / 65535.0; // doc page 13
    *humidity = 100.0 * (float)raw_humidity / 65535.0;
}




void app_main() {
    sh31_i2c_master_init();

    SSD1306_t dev;

	ESP_LOGI(tag, "INTERFACE is i2c");
	ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

	ESP_LOGI(tag, "Panel is 128x64");

	ssd1306_init(&dev, 128, 64);

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	//ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);
	//vTaskDelay(3000 / portTICK_PERIOD_MS);

    while (1) {
        float temperature, humidity;
        sht3x_read_data(&temperature, &humidity);

        
        char temperatureString[16]; // Adjust the size based on your needs
        char humidityString[16]; // Adjust the size based on your needs


        // Convert float to string with two decimal places
        sprintf(temperatureString, "%.2f", temperature);

        sprintf(humidityString, "%.2f", humidity);

        printf(temperatureString);
        printf(" : ");
        printf(humidityString);
        printf("\n");

        ssd1306_display_text_x3(&dev, 0, temperatureString, 16, false);
        ssd1306_display_text_x3(&dev, 5, humidityString, 16, false);


        vTaskDelay(1000 / portTICK_PERIOD_MS); // Pause for 1 second
    }
}
