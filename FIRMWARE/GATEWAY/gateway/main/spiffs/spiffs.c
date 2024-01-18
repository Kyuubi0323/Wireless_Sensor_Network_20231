/**
 * @file spiffs.c
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
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

static const char *TAG = "SPIFFS";

esp_err_t write_to_file(char *file_name, char *buf)
{
    char *base_path = "/spiffs";
    char file[64];
    sprintf(file, "%s/%s", base_path, file_name);
    FILE *f = NULL;
    f = fopen(file, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fputs(buf, f);
    fclose(f);
    return ESP_OK;
}

esp_err_t read_from_file(char *file_name, char *buf)
{
    char *base_path = "/spiffs";
    char file[64];
    char line[1024];
    sprintf(file, "%s/%s", base_path, file_name);
    FILE *f = NULL;
    f = fopen(file, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fgets(line, sizeof(line), f);
    fclose(f);
    char *pos = strchr(line, '\n');
    if (pos)
        *pos = '\0';
    strcpy(buf, line);
    // ESP_LOGI(TAG, "Read from file: %s", buf);
    return ESP_OK;
}

void mount_SPIFFS(void)
{
    ESP_LOGI(TAG, "SPIFFS init");
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 7,
        .format_if_mount_failed = true};
    esp_err_t ret = esp_vfs_spiffs_register(&spiffs_cfg);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        else
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
    }
}

void remove_file(char *file_name)
{
    struct stat st;
    if (stat(file_name, &st) == 0)
        unlink(file_name);
}
