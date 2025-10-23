idf_component_register(SRCS "Simpler.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver spiffs esp_adc esp_timer nvs_flash)
