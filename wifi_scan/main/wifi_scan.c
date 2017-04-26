/* WIFI Scan Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

xSemaphoreHandle print_mux;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
	ESP_LOGI("wifi_scan", "SYSTEM_EVENT_STA_START");
	ESP_ERROR_CHECK(esp_wifi_start());
	break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_FLASH) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void wifi_scan (void *pvParameter)

{
    uint16_t num;
    while(1)
    {
	    esp_wifi_scan_start(NULL,true);
	    esp_wifi_scan_get_ap_num(&num);
	    if(num == 0)
	    {
		    ESP_LOGI("wifi_scan", "Scanned %d APs \r\n", num);
		    wifi_ap_record_t *ap_records;
		    ap_records = malloc(num * sizeof(wifi_ap_record_t));
		    esp_wifi_scan_get_ap_records(&num, ap_records);
		    free(ap_records);
	    }

            else
            {
		    ESP_LOGI("wifi_scan", "Scanned %d APs \r\n", num);
		    wifi_ap_record_t *ap_records;
		    ap_records = malloc(num * sizeof(wifi_ap_record_t));
		    esp_wifi_scan_get_ap_records(&num, ap_records);
		    for (uint16_t i = 0; i < num; i++) {
			printf("%32s rssi: %02d\r\n", ap_records[i].ssid, ap_records[i].rssi);
		    }
		    free(ap_records);
            }
            esp_wifi_scan_stop();
	    ESP_LOGI("wifi_scan", "Restarting in 10 seconds!");
	    vTaskDelay(10000 / portTICK_PERIOD_MS);
	    ESP_LOGI("wifi_scan", "Starting again!");
    }
} 

void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    nvs_flash_init();

    initialise_wifi();
    xTaskCreate(&wifi_scan, "wifi_scan", 4096, NULL, 5, NULL);
}
