/**
 * @file common.c
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023

 */

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
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include "esp_http_client.h"
#include "esp_attr.h"
#include "esp_http_server.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"



#include "wifi/wifi_sta.h"
#include "wifi/wifi_ap.h"
#include "cfg/common.h"
#include "interface/button.h"
#include "interface/led.h"
#include "cjson/cJSON.h"

#include "spiffs/spiffs.h"
#include "web_server/web_server.h"

static const char *TAG = "COMMON";

esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj)
{
    cJSON *root = cJSON_Parse(mqtt_data);
    if (root == NULL)
        return ESP_FAIL;
    cJSON *cur_elem = NULL;
    cJSON_ArrayForEach(cur_elem, root)
    {
        if (cur_elem->string)
        {
            const char *cur_str = cur_elem->string;
            if (strcmp(cur_str, "action") == 0)
                memcpy(mqtt_obj->action, cur_elem->valuestring, strlen(cur_elem->valuestring) + 1);
            else if (strcmp(cur_str, "url") == 0)
                memcpy(mqtt_obj->url, cur_elem->valuestring, strlen(cur_elem->valuestring) + 1);
            else if (strcmp(cur_str, "state") == 0)
                mqtt_obj->state = cur_elem->valueint;
            else if (strcmp(cur_str, "unicast_addr") == 0)
                mqtt_obj->unicast_addr = cur_elem->valueint;
            else if (strcmp(cur_str, "timeout") == 0)
                mqtt_obj->timeout = cur_elem->valueint;
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}
