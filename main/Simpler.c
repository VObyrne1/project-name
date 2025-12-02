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
#include "stdlib.h"

static const char *TAG = "ADXL345";

#define I2C_MASTER_SCL_IO           9 //CONFIG_I2C_MASTER_SCL  9-1
#define I2C_MASTER_SDA_IO           8 //CONFIG_I2C_MASTER_SDA  8-0
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


#define H3LIS331DL_ADDR          0x18
#define H3LIS331DL_WHO_AM_I_REG   0x0F
#define H3LIS331DL_CTRL_REG1    0x20
#define H3LIS331DL_OUT_X_L    0x28
#define H3LIS331DL_OUT_Y_L    0x2A
#define H3LIS331DL_OUT_Z_L    0x2C
#define H3LIS331DL_WHO_AM_I_EXPECTED 0x32
#define H3LIS331DL_CTRL_REG4       0x23


#define ADXL345_DEVID_EXPECTED      0xE5
#define LSB_TO_G                    0.0039f  // 3.9 mg/LSB for ±2g range
/* H3LIS331DL datasheet values:
    - Full scale: ±100 g
    - Sensitivity: 780 mg / digit (i.e. 0.78 g per LSB)
    - Zero-g offset accuracy: ±1.5 g
    - Acceleration noise density (ODR 50Hz): 50 mg/√Hz (approx)
    These values are used to convert raw counts to g.
*/
#define H3LIS_SENSITIVITY_MG       780.0f
#define H3LIS_LSB_TO_G             (H3LIS_SENSITIVITY_MG / 1000.0f) /* 0.78 g/LSB */
/* ADXL345 configured to ±16g full-resolution */
#define ADXL_MAX_G                  16.0f
/* Hand over when ADXL reaches this fraction of its max (configurable) */
#define ADXL_HANDOVER_RATIO         0.90f

#define THRESH_SEVERE_G             10.0f
#define THRESH_MODERATE_G           5.0f
#define MAX_LOG_FILE_SIZE           (100 * 1024)  // 100 KB max log file size

static TimerHandle_t major_impact_timer;
static bool major_impact_active = false;
static volatile bool major_impact_cancelled_flag = false;
static const char *TAG1 = "spiffs_list";
static int impact_count = 0;
static float baseline_x = 0.0f;
static float baseline_y = 0.0f;
static float baseline_z = 0.0f;
static float baseline_mag = 0.0f;

/* Button hold timer for SPIFFS clear (requires 10 second hold) */
static TimerHandle_t button_hold_timer = NULL;
static bool button_held = false;




static esp_err_t adxl345_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t h3lis_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    uint8_t addr = reg_addr;
    /* Some devices require setting MSB for auto-increment during multi-byte read */
    if (len > 1) addr = reg_addr | 0x80;
    return i2c_master_transmit_receive(dev_handle, &addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t h3lis_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = { reg_addr, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void h3lis_init(i2c_master_dev_handle_t dev_handle)
{
    uint8_t who = 0;
    if (h3lis_register_read(dev_handle, H3LIS331DL_WHO_AM_I_REG, &who, 1) == ESP_OK) {
        ESP_LOGI(TAG, "H3LIS WHO_AM_I = 0x%02X", who);
        if (who != H3LIS331DL_WHO_AM_I_EXPECTED) {
            ESP_LOGW(TAG, "Unexpected H3LIS WHO_AM_I (expected 0x%02X)", H3LIS331DL_WHO_AM_I_EXPECTED);
        }
    } else {
        ESP_LOGW(TAG, "Failed to read H3LIS WHO_AM_I");
    }

    /* Configure H3LIS: enable X/Y/Z, set ODR to a high rate and normal mode.
       0x27: common value to enable axes and set ODR (adjust if needed).
       Also set CTRL_REG4 to select full-scale ±100g if required by the device.
    */
    if (h3lis_register_write_byte(dev_handle, H3LIS331DL_CTRL_REG1, 0x27) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write H3LIS CTRL_REG1");
    } else {
        ESP_LOGI(TAG, "H3LIS CTRL_REG1 configured");
    }

    /* Set full-scale to ±100g: value below sets FS bits for ±100g on many H3LIS331DL versions.
       If your module requires a different value for ±100g, adjust H3LIS_FS_VAL accordingly. */
    /* H3LIS FS register value for ±100g. This should be set to the value
       specified by the H3LIS331DL datasheet for your chosen FS setting.
       If you want me to set the exact datasheet value I can fetch it and
       update this constant; for now this is a placeholder that you can
       change after confirming with the datasheet. */
     //CTRL_REG4 value provided by user (0x23) — sets FS and related bits per module config 
    const uint8_t H3LIS_CTRL_REG4_FS_100G = 0x23; /* user provided */
    if (h3lis_register_write_byte(dev_handle, H3LIS331DL_CTRL_REG4, H3LIS_CTRL_REG4_FS_100G) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write H3LIS CTRL_REG4 (FS)");
    } else {
        ESP_LOGI(TAG, "H3LIS CTRL_REG4 (FS) configured (val=0x%02X)", H3LIS_CTRL_REG4_FS_100G);
    }
    ESP_LOGI(TAG, "H3LIS configured: FS=±100g, sensitivity=%.1f mg/LSB (%.3f g/LSB), zero-g offset accuracy=±1.5 g", H3LIS_SENSITIVITY_MG, H3LIS_LSB_TO_G);
}

/* Helper: read ADXL345 and return X/Y/Z in g (Z with gravity offset like rest of code)
   Returns true on success. */
static bool read_adxl_xyz(i2c_master_dev_handle_t dev_handle, float *xg, float *yg, float *zg){
    uint8_t data[6];
    if (adxl345_register_read(dev_handle, ADXL345_DATAX0_REG, data, 6) != ESP_OK) return false;
    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);
    *xg = x_raw * LSB_TO_G;
    *yg = y_raw * LSB_TO_G;
    *zg = z_raw * LSB_TO_G - 1.0f; // same gravity offset used elsewhere
    return true;
}

/* Helper: read H3LIS331DL and return X/Y/Z in g (apply same gravity offset to Z).
   Returns true on success. */
static bool read_h3lis_xyz(i2c_master_dev_handle_t dev_handle, float *xg, float *yg, float *zg)
{
    uint8_t data[6];
     //H3LIS registers OUT_X_L .. OUT_Z_H are contiguous in many modes; read 6 bytes starting at OUT_X_L 
    if (h3lis_register_read(dev_handle, H3LIS331DL_OUT_X_L, data, 6) != ESP_OK) return false;
    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);
    *xg = x_raw * H3LIS_LSB_TO_G;
    *yg = y_raw * H3LIS_LSB_TO_G;
    *zg = z_raw * H3LIS_LSB_TO_G - 1.0f; 
    return true;
}

static esp_err_t adxl345_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = { reg_addr, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle_adxl, i2c_master_dev_handle_t *dev_handle_h3lis)
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
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle_adxl));

    /* H3LIS support disabled - do not add H3LIS device */
    /* dev_config.device_address = H3LIS331DL_ADDR; */
    /* ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle_h3lis)); */
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

/* Timer callback: clears SPIFFS if button was held for full 10 seconds */
static void button_hold_timer_callback(TimerHandle_t xTimer)
{
    
    if (gpio_get_level(BUTTON_GPIO) == 0) {
        FILE *f = fopen("/spiffs/i2c_log.txt", "w");
        impact_count = 0;
        gpio_set_level(LED_GPIO2, 1);
        fclose(f);
        ESP_LOGI(TAG, "Log file cleared by 10-second button hold.");
        button_held = false;
    } else {
        ESP_LOGI(TAG, "Button released before 10 seconds; clear cancelled.");
        button_held = false;
    }
}

void log_data_to_spiffs(float mag, float x_g, float y_g, float z_g)
{
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


static void init_led(int gpio){
    gpio_config_t io_conf = {.pin_bit_mask=1ULL<<gpio,.mode=GPIO_MODE_OUTPUT,.pull_up_en=0,.pull_down_en=0,.intr_type=GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level(gpio,1);
}
/*
static void flash_led(int GPIO){
    for(int i=0;i<10;i++){
        gpio_set_level(GPIO,0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(GPIO,1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/
/* Non-blocking LED blink task — start_led_blink/stop_led_blink
   allows the LED to flash in background while main loop collects data.
*/
static TaskHandle_t led_blink_task_handle = NULL;
static int led_blink_gpio = -1;

typedef struct {
    int gpio;
    uint32_t on_ms;
    uint32_t off_ms;
} led_blink_params_t;

static void led_blink_task(void *arg)
{
    led_blink_params_t params = *(led_blink_params_t *)arg;
    free(arg);

    while (1) {
        gpio_set_level(params.gpio, 0); // on (active low in this board)
        vTaskDelay(pdMS_TO_TICKS(params.on_ms));
        gpio_set_level(params.gpio, 1); // off
        vTaskDelay(pdMS_TO_TICKS(params.off_ms));
    }
}

static void start_led_blink(int gpio, uint32_t on_ms, uint32_t off_ms)
{
    if (led_blink_task_handle != NULL) return; // already running
    led_blink_gpio = gpio;
    led_blink_params_t *p = malloc(sizeof(led_blink_params_t));
    if (!p) return;
    p->gpio = gpio;
    p->on_ms = on_ms;
    p->off_ms = off_ms;
    xTaskCreate(led_blink_task, "led_blink", 2048, p, tskIDLE_PRIORITY + 1, &led_blink_task_handle);
}

static void stop_led_blink(void)
{
    if (led_blink_task_handle == NULL) return;
    vTaskDelete(led_blink_task_handle);
    led_blink_task_handle = NULL;
    if (led_blink_gpio >= 0) {
        gpio_set_level(led_blink_gpio, 1); // ensure LED off
        led_blink_gpio = -1;
    }
}


#if defined(LEDC_HIGH_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_HIGH_SPEED_MODE
#elif defined(LEDC_LOW_SPEED_MODE)
#define BUZZER_LEDC_MODE LEDC_LOW_SPEED_MODE
#else
#define BUZZER_LEDC_MODE 0
#endif


static const char* infer_impact_type(float ax,float ay,float az,float mag){
    
    if(mag>THRESH_SEVERE_G && az<(-1.0f) && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Fall";
    if(mag>THRESH_MODERATE_G && fabsf(ax)>(5*baseline_x) && fabsf(ay)>(5*baseline_y) && fabsf(az)>(5*baseline_z)) return "Ceiling Collapse";
    if(mag>THRESH_SEVERE_G && az>1.0f && fabsf(az)>fabsf(ax) && fabsf(az)>fabsf(ay)) return "Hard Landing";
    if(mag>THRESH_MODERATE_G && ax>(5*baseline_x)) return "Front Impact";
    if(mag>THRESH_MODERATE_G && ax<(-5*baseline_x)) return "Rear Impact";
    if(mag>THRESH_MODERATE_G && ay>(5*baseline_y)) return "Right Side Impact";
    if(mag>THRESH_MODERATE_G && ay<(-5*baseline_y)) return "Left Side Impact";
    
    if(mag>THRESH_MODERATE_G) return "blunt";
    return "none";
}

/* Collect baseline accelerometer readings for duration_ms sampling every interval_ms.
   This blocks for the duration (intended to run at startup) and fills baseline globals.
*/
static void collect_baseline(i2c_master_dev_handle_t dev_handle, uint32_t duration_ms, uint32_t interval_ms)
{
    const int max_samples = (duration_ms + interval_ms - 1) / interval_ms;
    if (max_samples <= 0) return;

    float sx = 0.0f, sy = 0.0f, sz = 0.0f, sm = 0.0f;
    int samples = 0;
    uint8_t data[6];

    ESP_LOGI(TAG, "Collecting baseline for %u ms (%d samples)...", duration_ms, max_samples);
    for (int i = 0; i < max_samples; ++i) {
        float x_g, y_g, z_g;
        if (read_adxl_xyz(dev_handle, &x_g, &y_g, &z_g)) {
            float mag = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
            sx += fabsf(x_g);
            sy += fabsf(y_g);
            sz += fabsf(z_g);
            sm += mag;
            samples++;
        } else {
            ESP_LOGW(TAG, "Baseline sample %d failed", i);
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    if(samples > 0){
        baseline_x = sx / samples;
        baseline_y = sy / samples;
        baseline_z = sz / samples;
        baseline_mag = sm / samples;
        ESP_LOGI(TAG, "Baseline collected: X=%.3fg Y=%.3fg Z=%.3fg MAG=%.3fg (n=%d)", baseline_x, baseline_y, baseline_z, baseline_mag, samples);
    } 
    else {
        ESP_LOGW(TAG, "No baseline samples collected");
    }
}

static void classify_impact(float x_g, float y_g, float z_g){
    float mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
    if(x_g > THRESH_SEVERE_G || x_g < -THRESH_SEVERE_G || y_g > THRESH_SEVERE_G || y_g < -THRESH_SEVERE_G || z_g > THRESH_SEVERE_G || z_g < -THRESH_SEVERE_G){
            ESP_LOGW(TAG, "Major Impact Detected: Starting Timer!");
            if(!major_impact_active){
                major_impact_active=true;
                xTimerStart(major_impact_timer,0);
                
            }
            log_data_to_spiffs(mag, x_g, y_g, z_g);
            start_led_blink(LED_GPIO1, 100, 100);
            //buzzer_beep_ms(4000, 5000);

            
    }
    else if(x_g > THRESH_MODERATE_G || x_g < -THRESH_MODERATE_G || y_g > THRESH_MODERATE_G || y_g < -THRESH_MODERATE_G || z_g > THRESH_MODERATE_G || z_g < -THRESH_MODERATE_G){
            ESP_LOGW(TAG, "Minor Impact!");
            log_data_to_spiffs(mag, x_g, y_g, z_g);
            impact_count++;
    }
    const char* impact_type = infer_impact_type(x_g,y_g,z_g,mag);
    
    if(strcmp(impact_type, "none")!=0){
        ESP_LOGE(TAG, "Impact detected! Type: %s, Magnitude: %.3f g", impact_type, mag);
    }
   
    if(impact_count>=5 && mag>THRESH_MODERATE_G){
            ESP_LOGW(TAG, "Multiple impacts detected (%d)! Go see a doctor!", impact_count);
            //impact_count=0;
            gpio_set_level(LED_GPIO2,0);
        }
    //float mag=sqrtf(x_g*x_g+y_g*y_g+z_g*z_g);
}


static void IRAM_ATTR button_handler(void* arg){
    /* ISR: handle button press for both major impact cancel and SPIFFS clear hold. */
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
    
    /* Start the 10-second hold timer for SPIFFS clear if not already running */
    if (!button_held && button_hold_timer != NULL) {
        button_held = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerStartFromISR(button_hold_timer, &xHigherPriorityTaskWoken);
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
        stop_led_blink();
        return;
    }

    stop_led_blink();
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
    i2c_master_dev_handle_t dev_handle_adxl;
    i2c_master_dev_handle_t dev_handle_h3lis;
    // Initialize I2C and attach ADXL345 and H3LIS331DL
    i2c_master_init(&bus_handle, &dev_handle_adxl, &dev_handle_h3lis);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Verify ADXL345 identity
    esp_err_t ret = adxl345_register_read(dev_handle_adxl, ADXL345_DEVID_REG, &devid, 1);
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
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle_adxl, ADXL345_POWER_CTL_REG, 0x08));

    // Set data format: Full resolution, ±2g range (0x08)
    /* Set ADXL345 to full-resolution and ±16g range (FULL_RES=1, range=3 -> 0x0B) */
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle_adxl, ADXL345_DATA_FORMAT_REG, 0x0B));
    init_led(LED_GPIO1);
    init_led(LED_GPIO2);
    
    collect_baseline(dev_handle_adxl, 5000, 100);
    /* H3LIS support disabled: skip h3lis_init(dev_handle_h3lis); */
    //buzzer_init();

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
    
    // Create button hold timer (10 sec for SPIFFS clear)
    button_hold_timer = xTimerCreate("ButtonHoldTimer", pdMS_TO_TICKS(10000), pdFALSE, NULL, button_hold_timer_callback);
    
    init_spiffs();


    while (1) {
        // Read ADXL345 first (default sensor)
        float x_g = 0.0f, y_g = 0.0f, z_g = 0.0f;
        if (!read_adxl_xyz(dev_handle_adxl, &x_g, &y_g, &z_g)) {
            ESP_LOGW(TAG, "ADXL read failed; skipping sample");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float mag = sqrtf(x_g*x_g + y_g*y_g + z_g*z_g);

        /* If ADXL indicates a high-G event above 1.5g, query the H3LIS
           for a more accurate high-G measurement and use that for
           classification when available. */
        /* Hand over to H3LIS when ADXL reaches its maximum measurable g (ADXL_MAX_G).
           This ensures we rely on the high-G sensor only when ADXL is beyond its range. */
        /* Hand over to H3LIS when ADXL reaches 90% of its maximum measurable g.
           This gives an early handover to the high-g sensor. */
       
        /*if (mag >= (ADXL_HANDOVER_RATIO * ADXL_MAX_G)) {
            ESP_LOGI(TAG, "H3LIS support disabled at compile-time; using ADXL only (mag=%.3f)", mag);
        }
        */


        classify_impact(x_g, y_g, z_g);
        

        /* If the button cancelled a major impact warning in the ISR,
           perform the non-ISR-safe logging here in task context. */
        if (major_impact_cancelled_flag) {
            major_impact_cancelled_flag = false;
            ESP_LOGI(TAG, "Major impact warning cancelled by button!");
            stop_led_blink();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }


    // Clean up (never reached in this loop)
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle_adxl));
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle_h3lis));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}
