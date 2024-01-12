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

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_http_client.h"
#include "esp_attr.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"

#include "driver/gpio.h"

#include "cjson/cJSON.h"
#include "cjson/cJSON_Utils.h"
#include "mqtt/mqtt.h"
#include "smartcfg/smartcfg.h"
#include "wifi/wifi_sta.h"
#include "wifi/wifi_ap.h"
#include "cfg/common.h"
#include "interface/button.h"
#include "interface/led.h"
#include "fota/fota.h"
#include "mesh/ble_mesh_user.h"
#include "spiffs/spiffs.h"
#include "web_server/web_server.h"

static const char *TAG = "MAIN";
RTC_NOINIT_ATTR int gateway_mode_flag;
char version[10] = "0.1.0";
char topic_commands_set[50] = "mandevices/commands/set";
char topic_commands_get[50] = "mandevices/commands/get";
char topic_commands_status[50] = "mandevices/commands/status";
char topic_commands_heartbeat_node[50] = "mandevices/commands/heartbeat/node";
char topic_commands_heartbeat_gateway[50] = "mandevices/commands/heartbeat/gateway";
char topic_commands_network[50] = "mandevices/commands/network";
char topic_commands_process[50] = "mandevices/commands/process";
char topic_commands_version[50] = "mandevices/commands/version";
char topic_commands_fota[50] = "mandevices/commands/fota";
char topic_commands_group[50] = "mandevices/commands/group";
char topic_commands_provision[50] = "mandevices/commands/provision";
status_red_t status_red = LOCAL_MODE;
status_blue_t status_blue = POWER_ON_PROVISIONING;
TaskHandle_t prov_dev_handle;
uint8_t dev_uuid[16];
TimerHandle_t hb_gateway_timer;
esp_mqtt_client_handle_t client;
RingbufHandle_t webserver_ring_buf;
httpd_handle_t server;

void gateway_mesh_init(void)
{
    esp_err_t err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}

void hb_gateway_timer_cb(TimerHandle_t hb_gateway_timer)
{
    static int sec = 0;
    char buf[50] = {0};
    sec += 30;
    if (status_red == NORMAL_MODE)
    {
        sprintf(buf, "{\"action\":\"hb_gateway\",\"runtime\":%d}", sec);
        esp_mqtt_client_publish(client, topic_commands_heartbeat_gateway, buf, strlen(buf), 0, 0);
    }
}

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
    hb_gateway_timer = xTimerCreate("Heartbeat Gateway", 30000 / portTICK_RATE_MS, pdTRUE, (void *)0, hb_gateway_timer_cb);
    xTimerStart(hb_gateway_timer, 0);
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
                vTaskDelay(2000 / portTICK_RATE_MS);
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
            mqtt_client_sta();
            gateway_mesh_init();
        }
    }
}