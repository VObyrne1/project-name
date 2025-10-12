#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_err.h"
#include "string.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "esp_mac.h"

static const char *TAG = "ADXL345";

#define I2C_MASTER_SCL_IO           9 //CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO           8 //CONFIG_I2C_MASTER_SDA
#define LED_GPIO                    2 //CONFIG_LED_GPIO
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

#define THRESH_SEVERE_G             1.0f
#define THRESH_MODERATE_G           0.5f


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

static void init_led(void){
    gpio_config_t io_conf = {.pin_bit_mask=1ULL<<LED_GPIO,.mode=GPIO_MODE_OUTPUT,.pull_up_en=0,.pull_down_en=0,.intr_type=GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO,1);
}

static void flash_led(int GPIO){
    for(int i=0;i<5;i++){
        gpio_set_level(GPIO,1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO,0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static const char* infer_impact_type(float ax,float ay,float az,float mag){
    if(mag>THRESH_SEVERE_G && az<-1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "fall";
    if(mag>THRESH_MODERATE_G && fabsf(ax)>1.0f && fabsf(ay)>1.0f && fabsf(az)>1.0f) return "collapse";
    if(mag>THRESH_MODERATE_G) return "blunt";
    return "none";
}

static void classify_impact(float x_g, float y_g, float z_g, float mag){
    if(x_g > THRESH_MODERATE_G || x_g < -THRESH_MODERATE_G || y_g > THRESH_MODERATE_G || y_g < -THRESH_MODERATE_G || z_g > THRESH_MODERATE_G || z_g < -THRESH_MODERATE_G){
            ESP_LOGW(TAG, "Minor Impact!");
    }
    if(x_g > THRESH_SEVERE_G || x_g < -THRESH_SEVERE_G || y_g > THRESH_SEVERE_G || y_g < -THRESH_SEVERE_G || z_g > THRESH_SEVERE_G || z_g < -THRESH_SEVERE_G){
            ESP_LOGW(TAG, "Major Impact!");
            flash_led(LED_GPIO);
    }
        
    mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
    const char* impact_type = infer_impact_type(x_g,y_g,z_g,mag);
    if(impact_type!="none"){
        ESP_LOGE(TAG, "Impact detected! Type: %s, Magnitude: %.3f g", impact_type, mag);
    }
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
    init_led();

    while (1) {
        // Read 6 bytes of acceleration data
        ESP_ERROR_CHECK(adxl345_register_read(dev_handle, ADXL345_DATAX0_REG, data, 6));
        
        //gpio_set_level(LED_GPIO,1);
    
        int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
        int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
        int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

        float x_g = x_raw * LSB_TO_G;
        float y_g = y_raw * LSB_TO_G;
        float z_g = z_raw * LSB_TO_G-1.0f;//gravity 
        
        
        ESP_LOGI(TAG, "X=%.3f g, Y=%.3f g, Z=%.3f g", x_g, y_g, z_g);
        classify_impact(x_g, y_g, z_g);
        /*
        ESP_LOGI(TAG, "X=%.3f g, Y=%.3f g, Z=%.3f g", x_g, y_g, z_g);
        if(x_g > 0.5f || x_g < -0.5f || y_g > 0.5f || y_g < -0.5f || z_g > 0.5f || z_g < -0.5f){
            ESP_LOGW(TAG, "Minor Impact!");
        }
        if(x_g > 1.0f || x_g < -1.0f || y_g > 1.0f || y_g < -1.0f || z_g > 1.0f || z_g < -1.0f){
            ESP_LOGW(TAG, "Major Impact!");
            flash_led(LED_GPIO);
        }
        
        float mag = sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
        const char* impact_type = infer_impact_type(x_g,y_g,z_g,mag);
        if(impact_type!="none"){
            ESP_LOGE(TAG, "Impact detected! Type: %s, Magnitude: %.3f g", impact_type, mag);
        }*/
        

        vTaskDelay(pdMS_TO_TICKS(100));
    }


    // Clean up (never reached in this loop)
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}

