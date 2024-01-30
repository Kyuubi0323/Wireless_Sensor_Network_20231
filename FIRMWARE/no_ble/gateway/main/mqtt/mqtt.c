/**
 * @file mqtt.c
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
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
#include "cjson/cJSON.h"


#include "mqtt/mqtt.h"
#include "smartcfg/smartcfg.h"
#include "wifi/wifi_sta.h"
#include "wifi/wifi_ap.h"
#include "cfg/common.h"
#include "interface/button.h"
#include "interface/led.h"

#include "spiffs/spiffs.h"
#include "web_server/web_server.h"


static const char *TAG = "MQTT";
RingbufHandle_t mqtt_ring_buf;
extern esp_mqtt_client_handle_t client;
extern char version[10];
extern char topic_commands_set[50];
extern char topic_commands_get[50];
extern char topic_commands_status[50];
extern char topic_commands_heartbeat[50];
extern char topic_commands_network[50];
extern char topic_commands_process[50];
extern char topic_commands_version[50];
extern char topic_commands_fota[50];
extern char topic_commands_provision[50];
extern char topic_commands_data[50];
extern char topic_commands_TB[50];
extern status_red_t status_red;
extern status_blue_t status_blue;
extern node_t node[10];

node[0].temp = 1800;


static void cJSON_mqtt_handler(esp_mqtt_event_handle_t *event_data)
{   esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    
    // cJSON *value = cJSON_CreateObject();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temp", node[0].temp);
    cJSON_AddNumberToObject(root, "batt", node[0].battery);
    cJSON_AddStringToObject(root, "status", node[0].status);
    cJSON_AddNumberToObject(root, "rssi", node[0].rssi);
    cJSON_AddNumberToObject(root, "snr", 12);
    char *payload = cJSON_Print(root);
    msg_id = esp_mqtt_client_publish(client, topic_commands_data, payload, 0, 1, 1);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    cJSON_Delete(root);
    if (payload != NULL)
    free(payload);

}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        char ver_json[50] = {0};
        status_red = NORMAL_MODE;
        
        ESP_LOGI(TAG, "MQTT event connected");
        esp_mqtt_client_subscribe(client, topic_commands_set, 0);
        esp_mqtt_client_subscribe(client, topic_commands_fota, 0);
        esp_mqtt_client_subscribe(client, topic_commands_get, 0);
        esp_mqtt_client_subscribe(client, topic_commands_provision, 0);
        
        esp_mqtt_client_subscribe(client, topic_commands_data, 0);
        sprintf(ver_json, "{\"action\":\"version\",\"firm_ver\":\"%s\"}", version);
        esp_mqtt_client_publish(client, topic_commands_version, ver_json, strlen(ver_json), 0, 0);
        

        cJSON_mqtt_handler(event);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        status_red = LOCAL_MODE;
        ESP_LOGI(TAG, "MQTT event disconnected");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event subcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event unsubcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT event published, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
    {   
        UBaseType_t res = xRingbufferSend(mqtt_ring_buf, event->data, event->data_len, portMAX_DELAY);
        if(res != pdTRUE)
            ESP_LOGE(TAG, "Failed to send item\n");
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT event error");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
void mqtt_client_task(void *param)
{
    char *mess_recv = NULL;
    size_t mess_size = 0;
    mqtt_obj_t mqtt_obj;
    //esp_ble_mesh_client_common_param_t common;
    while(1)
    {
        mess_recv = (char*)xRingbufferReceive(mqtt_ring_buf, &mess_size, portMAX_DELAY);
        if (mess_recv)
        {
            mess_recv[mess_size] = '\0';
            ESP_LOGI(TAG, "Recv payload: %s", mess_recv);
            memset(&mqtt_obj, 0, sizeof(mqtt_obj));
            mqtt_parse_data(mess_recv, &mqtt_obj);
            if (strcmp(mqtt_obj.action, "set") == 0)
            {
                //ble_mesh_onoff_set_state(&common, (uint16_t)mqtt_obj.unicast_addr, onoff_client.model, (uint8_t)mqtt_obj.state);
            }
            else if (strcmp(mqtt_obj.action, "upgrade") == 0)
            {
                uint32_t free_heap_size = 0, min_free_heap_size = 0;
                free_heap_size = esp_get_free_heap_size();
                min_free_heap_size = esp_get_minimum_free_heap_size();
                ESP_LOGW(TAG, "Free heap size = %d, Min free heap size = %d", free_heap_size, min_free_heap_size);
                //ble_mesh_deinit();
                //xTaskCreate(&fota_task, "fota_task", 8192, mqtt_obj.url, 8, NULL);
            }
            else if (strcmp(mqtt_obj.action, "open") == 0)
            {
                ESP_LOGI(TAG, "here is open");
                status_blue = PROVISIONING;
            }
            else if (strcmp(mqtt_obj.action, "close") == 0)
            {
                status_blue = NOT_STATE;
            }
            else if (strcmp(mqtt_obj.action, "delete") == 0)
            {
                //esp_ble_mesh_client_common_param_t common = {0};
                //node_info_t *node = ble_mesh_get_node_info_with_unicast(mqtt_obj.unicast_addr);
                //ble_mesh_config_node_reset(&common, node, config_client.model);
            }
            else if (strcmp(mqtt_obj.action, "set_timeout") == 0)
            {
        
            }
            vRingbufferReturnItem(mqtt_ring_buf, (void*)mess_recv);
        }
    }
}

void mqtt_client_start(void)
{   
    uint8_t broker[50] = {0};
    ESP_LOGI(TAG, "MQTT init");
    ESP_LOGI(TAG, "Broker: %s", MQTT_BROKER);
    sprintf((char*)broker, "mqtt://%s", MQTT_BROKER);
    mqtt_ring_buf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    if (mqtt_ring_buf == NULL)
        ESP_LOGE(TAG, "Failed to create ring buffer");
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = (char *)broker,
        .broker.address.port = 1883,
        // .credentials.username = MQTT_USERNAME,
        // .credentials.client_id = MQTT_PASSWORD,
        .session.keepalive = 60,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    xTaskCreate(&mqtt_client_task, "mqtt_task", 4096, NULL, 9, NULL);
}

