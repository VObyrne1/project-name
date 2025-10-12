#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "ADXL345";

#define I2C_MASTER_SCL_IO           9 //CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO           8 //CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADXL345_ADDR                0x53
#define ADXL345_DEVID_REG           0x00
#define ADXL345_POWER_CTL_REG       0x2D
#define ADXL345_DATA_FORMAT_REG     0x31
#define ADXL345_DATAX0_REG          0x32

#define ADXL345_DEVID_EXPECTED      0xE5
#define LSB_TO_G                    0.0039f  // 3.9 mg/LSB for ±2g range

static esp_err_t adxl345_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t adxl345_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = { reg_addr, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}


void app_main(void)
{
    uint8_t data[6];
    uint8_t devid = 0;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    // Initialize I2C and attach ADXL345
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Verify ADXL345 identity
    esp_err_t ret = adxl345_register_read(dev_handle, ADXL345_DEVID_REG, &devid, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADXL345 DEVID = 0x%02X", devid);
        if (devid != ADXL345_DEVID_EXPECTED) {
            ESP_LOGE(TAG, "ADXL345 not detected! Expected 0xE5.");
            return;
        }
    } else {
        ESP_LOGE(TAG, "Failed to communicate with ADXL345 at 0x%02X", ADXL345_ADDR);
        return;
    }

    // Enable measurement mode
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle, ADXL345_POWER_CTL_REG, 0x08));

    // Set data format: Full resolution, ±2g range (0x08)
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle, ADXL345_DATA_FORMAT_REG, 0x08));

    while (1) {
        // Read 6 bytes of acceleration data
        ESP_ERROR_CHECK(adxl345_register_read(dev_handle, ADXL345_DATAX0_REG, data, 6));

        int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
        int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
        int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

        float x_g = x_raw * LSB_TO_G;
        float y_g = y_raw * LSB_TO_G;
        float z_g = z_raw * LSB_TO_G-1.0f;//gravity 

        ESP_LOGI(TAG, "X=%.3f g, Y=%.3f g, Z=%.3f g", x_g, y_g, z_g);
        if(x_g > 0.5f || x_g < -0.5f || y_g > 0.5f || y_g < -0.5f || z_g > 0.5f || z_g < -0.5f){
            ESP_LOGW(TAG, "Minor Impact!");
        }
        if(x_g > 1.0f || x_g < -1.0f || y_g > 1.0f || y_g < -1.0f || z_g > 1.0f || z_g < -1.0f){
            ESP_LOGW(TAG, "Major Impact!");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clean up (never reached in this loop)
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}

