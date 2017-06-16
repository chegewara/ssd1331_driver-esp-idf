#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp_log.h>

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

extern void ssd1331_test(void *ignore);

void app_main(void)
{
	ESP_LOGI("tag", ">> app_main");
    xTaskCreatePinnedToCore(&ssd1331_test, "ssd1331_test", 8048, NULL, 5, NULL, 0);
    ESP_LOGI("tag", ">> app_main");

}

