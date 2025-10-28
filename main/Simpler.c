#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
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
#include "esp_spiffs.h"
#include "dirent.h"

static const char *TAG = "ADXL345";

#define I2C_MASTER_SCL_IO           9 //CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA_IO           8 //CONFIG_I2C_MASTER_SDA
#define LED_GPIO1                   12//CONFIG_LED_GPIO
#define LED_GPIO2                   13 //CONFIG_LED_GPIO

#define BUTTON_GPIO                 15 //CONFIG_BUTTON_GPIO
#define BUZZER_GPIO                 20 //CONFIG_BUZZER_GPIO

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
#define MAX_LOG_FILE_SIZE           (100 * 1024)  // 100 KB max log file size

static TimerHandle_t major_impact_timer;
static bool major_impact_active = false;
static volatile bool major_impact_cancelled_flag = false;
static const char *TAG1 = "spiffs_list";
static int impact_count = 0;




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


static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs", //determining path for NVM
        .partition_label = "spiffs", //labeling at SPIFFS
        .max_files = 5,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total: %d bytes, used: %d bytes", total, used);
    } else {
        ESP_LOGE(TAG, "SPIFFS info failed (%s)", esp_err_to_name(ret));
    }
}

void list_spiffs_files(void)
{
    DIR *dir = opendir("/spiffs");
    if (dir == NULL) {
        ESP_LOGE(TAG1, "Failed to open directory");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG1, "Found file: %s", entry->d_name);
    }

    closedir(dir);
}


void read_log_file(const char *file_path)
{
    FILE *f = fopen(file_path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open");
        return;
    }

    ESP_LOGI(TAG, "Reading log file: %s", file_path);

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        // Print each line to the console/log
        printf("%s", line);
    }

    fclose(f);
}

void log_data_to_spiffs(float mag, float x_g, float y_g, float z_g)
{
    if(gpio_get_level(BUTTON_GPIO)==0){
        FILE *f = fopen("/spiffs/i2c_log.txt", "w");
        impact_count=0;
        gpio_set_level(LED_GPIO2,1);
    }
    else{
    FILE *f = fopen("/spiffs/i2c_log.txt", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

        fprintf(f, "Magnitude: %f, X-G: %f, Y-G: %f, Z-G: %f \n",
                mag,
                x_g,
                y_g,
                z_g);
    fclose(f);

    read_log_file("/spiffs/i2c_log.txt");
    }
}


static void init_led(int gpio){
    gpio_config_t io_conf = {.pin_bit_mask=1ULL<<gpio,.mode=GPIO_MODE_OUTPUT,.pull_up_en=0,.pull_down_en=0,.intr_type=GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(gpio,1);
}

static void flash_led(int GPIO){
    for(int i=0;i<50;i++){
        gpio_set_level(GPIO,0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO,1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/* Buzzer using LEDC hardware PWM to avoid blocking the main task.
   We create a short one-shot FreeRTOS timer to stop the PWM after the
   requested duration so buzzer_beep_ms() is non-blocking.
*/
static const ledc_channel_t BUZZER_LEDC_CHANNEL = LEDC_CHANNEL_0;
static const ledc_timer_bit_t BUZZER_DUTY_RES = LEDC_TIMER_8_BIT;
static const ledc_timer_t BUZZER_LEDC_TIMER = LEDC_TIMER_0;

/* Select LEDC speed mode; fall back to 0 if symbol not defined for this target */
#if defined(LEDC_HIGH_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_HIGH_SPEED_MODE
#elif defined(LEDC_LOW_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#else
#define BUZZER_LEDC_MODE 0
#endif

static void buzzer_stop_timer_cb(TimerHandle_t xTimer)
{
    /* Stop PWM and delete this one-shot timer */
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    xTimerDelete(xTimer, 0);
}

static void buzzer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .duty_resolution  = BUZZER_DUTY_RES,
        .timer_num        = BUZZER_LEDC_TIMER,
        .freq_hz          = 2000, // default, will be updated per beep
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = BUZZER_GPIO,
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void buzzer_beep_ms(uint32_t freq_hz, uint32_t duration_ms)
{
    /* Configure frequency and start PWM at 50% duty */
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    uint32_t max_duty = (1 << BUZZER_DUTY_RES) - 1;
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, max_duty / 2);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);

    /* Create a one-shot timer to stop the buzzer after duration_ms */
    TimerHandle_t t = xTimerCreate("bstop", pdMS_TO_TICKS(duration_ms), pdFALSE, NULL, buzzer_stop_timer_cb);
    if (t) {
        xTimerStart(t, 0);
    }
}

static const char* infer_impact_type(float ax,float ay,float az,float mag){
    
    if(mag>THRESH_SEVERE_G && az<-1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Fall";
    if(mag>THRESH_MODERATE_G && fabsf(ax)>0.5f && fabsf(ay)>0.5f && fabsf(az)>0.5f) return "Ceiling Collapse";
    if(mag>THRESH_SEVERE_G && az>1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Hard Landing";
    if(mag>THRESH_MODERATE_G && ax>0.5f) return "Front Impact";
    if(mag>THRESH_MODERATE_G && ax<-0.5f) return "Rear Impact";
    if(mag>THRESH_MODERATE_G && ay>0.5f) return "Right Side Impact";
    if(mag>THRESH_MODERATE_G && ay<-0.5f) return "Left Side Impact";
    
    if(mag>THRESH_MODERATE_G) return "blunt";
    return "none";
}

static void classify_impact(float x_g, float y_g, float z_g){
    float mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
    if(x_g > THRESH_SEVERE_G || x_g < -THRESH_SEVERE_G || y_g > THRESH_SEVERE_G || y_g < -THRESH_SEVERE_G || z_g > THRESH_SEVERE_G || z_g < -THRESH_SEVERE_G){
            ESP_LOGW(TAG, "Major Impact Detected: Starting Timer!");
            if(!major_impact_active){
                major_impact_active=true;
                xTimerStart(major_impact_timer,0);
                log_data_to_spiffs(mag, x_g, y_g, z_g);
            }
            
            flash_led(LED_GPIO1);
            //buzzer_beep_ms(4000, 5000);
            //gpio_set_level(LED_GPIO1,0); //ensure LED off after flash

            
    }
    else if(x_g > THRESH_MODERATE_G || x_g < -THRESH_MODERATE_G || y_g > THRESH_MODERATE_G || y_g < -THRESH_MODERATE_G || z_g > THRESH_MODERATE_G || z_g < -THRESH_MODERATE_G){
            ESP_LOGW(TAG, "Minor Impact!");
            log_data_to_spiffs(mag, x_g, y_g, z_g);
            impact_count++;
    }
    
        
    if(impact_count>=5 && mag>THRESH_MODERATE_G){
            ESP_LOGW(TAG, "Multiple impacts detected (%d)! Go see a doctor!", impact_count);
            //impact_count=0;
            gpio_set_level(LED_GPIO2,0);
        }
    //float mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
    const char* impact_type = infer_impact_type(x_g,y_g,z_g,mag);
    if(strcmp(impact_type, "none")!=0){
        ESP_LOGE(TAG, "Impact detected! Type: %s, Magnitude: %.3f g", impact_type, mag);
    }
}

/*
void log_to_spiffs(float mag){
fopen(
fwrite(
fclose(
}
*/
static void IRAM_ATTR button_handler(void* arg){
    /* ISR: avoid calling non-ISR safe APIs (like ESP_LOG) here.
       Stop the timer from ISR and set a flag so the main task can log.
    */
    if (major_impact_active) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t stopped = xTimerStopFromISR(major_impact_timer, &xHigherPriorityTaskWoken);
        if (stopped == pdPASS) {
            /* Only mark cancelled if the timer was actually stopped */
            major_impact_active = false;
            major_impact_cancelled_flag = true;
        }
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void major_impact_timer_callback(TimerHandle_t xTimer){
    /* If cancellation was requested (set from ISR), skip executing the
       warning actions. The flag is volatile and may be set from the ISR.
    */
    if (major_impact_cancelled_flag) {
        /* Clear the flag here and don't perform the warning */
        major_impact_cancelled_flag = false;
        major_impact_active = false;
        return;
    }

    ESP_LOGW(TAG, "Major impact! WARNING!");
    //flash_led(LED_GPIO);
    gpio_set_level(LED_GPIO1,0);
    major_impact_active = false;
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
    init_led(LED_GPIO1);
    init_led(LED_GPIO2);
    buzzer_init();

    // Configure button input
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // enable pull-up
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE, // falling edge
    };
    gpio_config(&io_conf);

    // Install ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_handler, NULL);

    // Create major impact timer (15 sec)
    major_impact_timer = xTimerCreate("MajorImpactTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, major_impact_timer_callback);
    init_spiffs();


    while (1) {
        // Read 6 bytes of acceleration data
        ESP_ERROR_CHECK(adxl345_register_read(dev_handle, ADXL345_DATAX0_REG, data, 6));
        
        //gpio_set_level(LED_GPIO,1);
    
        int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
        int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
        int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

        float x_g = x_raw * LSB_TO_G;
        float y_g = y_raw * LSB_TO_G;
        float z_g = z_raw * LSB_TO_G -1.0f;//gravity 
        
        
        //ESP_LOGI(TAG, "X=%.3f g, Y=%.3f g, Z=%.3f g", x_g, y_g, z_g);
        classify_impact(x_g, y_g, z_g);
        

        /* If the button cancelled a major impact warning in the ISR,
           perform the non-ISR-safe logging here in task context. */
        if (major_impact_cancelled_flag) {
            major_impact_cancelled_flag = false;
            ESP_LOGI(TAG, "Major impact warning cancelled by button!");
            gpio_set_level(LED_GPIO1,1); 
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }


    // Clean up (never reached in this loop)
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}

