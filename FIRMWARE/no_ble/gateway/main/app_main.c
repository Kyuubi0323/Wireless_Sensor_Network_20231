#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"


#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_http_client.h"
#include "esp_attr.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"

#include "driver/gpio.h"

#include "mqtt/mqtt.h"
#include "smartcfg/smartcfg.h"
#include "wifi/wifi_sta.h"
#include "wifi/wifi_ap.h"
#include "cfg/common.h"
#include "interface/button.h"
#include "interface/led.h"
#include "cjson/cJSON.h"


#include "spiffs/spiffs.h"
#include "web_server/web_server.h"

static const char *TAG = "MAIN";
RTC_NOINIT_ATTR int gateway_mode_flag;
char version[10] = "2.0.0";
char topic_commands_set[50] = "MANDev/cmd/set";
char topic_commands_get[50] = "MANDev/cmd/get";
char topic_commands_status[50] = "MANDev/cmd/status";
char topic_commands_heartbeat_node[50] = "MANDev/cmd/heartbeat/node";
char topic_commands_heartbeat_gateway[50] = "MANDev/cmd/heartbeat/gateway";
char topic_commands_network[50] = "MANDev/cmd/network";
char topic_commands_process[50] = "MANDev/cmd/process";
char topic_commands_version[50] = "MANDev/cmd/version";
char topic_commands_fota[50] = "MANDev/cmd/fota";
char topic_commands_group[50] = "MANDev/cmd/group";
char topic_commands_provision[50] = "MANDev/cmd/provision";
char topic_commands_data[50] = "MANDev/cmd/data";
char topic_commands_TB[50] = "MANDev/cmd/pub";
status_red_t status_red = LOCAL_MODE;
status_blue_t status_blue = POWER_ON_PROVISIONING;
TaskHandle_t prov_dev_handle;
uint8_t dev_uuid[16];
TimerHandle_t hb_gateway_timer;
esp_mqtt_client_handle_t client;
RingbufHandle_t webserver_ring_buf;
httpd_handle_t server;
node_t node[10];


void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    mount_SPIFFS();
    xTaskCreate(&led_red_task, "led_red_task", 2048, NULL, 5, NULL);
    xTaskCreate(&led_blue_task, "led_blue_task", 2048, NULL, 5, NULL);
    xTaskCreate(&button_task, "button_task", 2048, NULL, 15, NULL);
    wifi_init();


    if (gateway_mode_flag == SMARTCONFIG_MODE)
    {
        gateway_mode_flag = MESH_MODE;
        status_blue = SMARTCONFIG;
        status_red = CONFIG_MODE;
        smartconfig_init();
    }
    else if (gateway_mode_flag == WIFI_SOFTAP_MODE)
    {
        char *buf_recv = NULL;
        size_t buf_size = 0;
        char ssid[50] = {0};
        char password[50] = {0};
        wifi_config_t wifi_config;
        webserver_ring_buf = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
        gateway_mode_flag = MESH_MODE;
        status_blue = WIFI_SOFTAP;
        status_red = CONFIG_MODE;
        wifi_init_softap();
        while (1)
        {
            buf_recv = (char *)xRingbufferReceive(webserver_ring_buf, &buf_size, portMAX_DELAY);
            if (buf_recv)
            {
                buf_recv[buf_size] = '\0';
                sscanf(buf_recv, ",%[^,],%[^,],", ssid, password);
                ESP_LOGI(TAG, "SSID: %s. Password: %s", ssid, password);
                bzero(&wifi_config, sizeof(wifi_config_t));
                memcpy(wifi_config.sta.ssid, ssid, strlen(ssid));
                memcpy(wifi_config.sta.password, password, strlen(password));
                stop_webserver(server);
                esp_wifi_stop();
                esp_wifi_set_mode(WIFI_MODE_STA);
                ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
                vRingbufferReturnItem(webserver_ring_buf, (void *)buf_recv);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                esp_restart();
            }
        }
    }
    else
    {
        wifi_config_t wifi_cfg = {
            .sta = {
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg = {
                    .capable = true,
                    .required = false,
                },
            },
        };
        if (esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg) == ESP_OK)
        {
            ESP_LOGI(TAG, "Wifi configuration already stored in flash partition called NVS");
            ESP_LOGI(TAG, "%s", wifi_cfg.sta.ssid);
            ESP_LOGI(TAG, "%s", wifi_cfg.sta.password);
            wifi_sta(wifi_cfg, WIFI_MODE_STA);
            mqtt_client_start();
            //gateway_mesh_init();
        }
    }
   
}