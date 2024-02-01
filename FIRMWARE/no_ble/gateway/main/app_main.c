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

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "ble_mesh_example_init.h"


#define MSG_SEND_TTL        3
#define MSG_SEND_REL        false
#define MSG_TIMEOUT         0
#define MSG_ROLE            ROLE_NODE

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[ESP_BLE_MESH_OCTET16_LEN];
} prov_key;

// static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t sensor_client;
static esp_ble_mesh_client_t sensor_client_1;

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_cli_pub, 20, MSG_ROLE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    // ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_CLI(&sensor_cli_pub, &sensor_client),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_cli_pub_1, 20, MSG_ROLE);

static esp_ble_mesh_model_t extend_model_level[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_CLI(&sensor_cli_pub_1, &sensor_client_1),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_level, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid
};

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


static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{

    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);

}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {

    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        // mesh_example_info_restore(); /* Restore proper mesh example info */
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        ESP_LOGI(TAG, "GATEWAY Provisioned Successfully");
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;

    }
}

// void example_ble_mesh_send_sensor_message(uint32_t opcode)
// {
//     esp_ble_mesh_light_client_get_state_t get = {0};
//     esp_ble_mesh_client_common_param_t common = {0};
//     esp_ble_mesh_node_t *node = NULL;
//     esp_err_t err = ESP_OK;

//     example_ble_mesh_set_msg_common(&common, node, sensor_client.model, opcode);
//     switch (opcode) {
//     case ESP_BLE_MESH_MODEL_OP_LIGHT_LC_PROPERTY_GET:
//         get.lc_property_get.property_id = sensor_prop_id;
//         break;
//     default:
//         break;
//     }

//     err = esp_ble_mesh_sensor_client_get_state(&common, &get);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to send sensor message 0x%04" PRIx32, opcode);
//     }
// }

static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_light_client_cb(esp_ble_mesh_light_client_cb_event_t event,
                                            esp_ble_mesh_light_client_cb_param_t *param)
{
    esp_ble_mesh_node_t *node = NULL;
    ESP_LOGI(TAG, "SENSOR CLIENT, event 0x%04x, addr: %d, RSSI:%d", event, param->params->ctx.addr, param->params->ctx.recv_rssi);
    if(param->status_cb.ctl_status.present_ctl_lightness)
    {
        ESP_LOGI(TAG, "TEMPERATURE NODE: %.2f°C", (float)((param->status_cb.ctl_status.present_ctl_lightness )/100.0) );
    }

    if(param->error_code)
    {
        ESP_LOGE(TAG, "Get lightness client message failed (err %d)", param->error_code);
        return;
    }

    // switch(event)
    // {
    //     case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
    //         ESP_LOGI(TAG, "ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT");
    //         if(param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET)
    //         {
    //             ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET, lightness: %d", param->status_cb.lightness_status.present_lightness);
    //         }
    //         break;
    //     case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
    //         ESP_LOGI(TAG,"ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT" );
    //         if(param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET)
    //         {
    //             ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET, lightness: %d", param->status_cb.lightness_status.target_lightness);
    //         }
    //         break;
    //     case ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT:
    //         ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_CLIENT_PUBLISH_EVT");
    //         break;
    //     case ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT:
    //         ESP_LOGI(TAG, "ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT");
    //         break;
    //     default:
    //         break;
    // }
}

static void exmaple_ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                               esp_ble_mesh_generic_client_cb_param_t *param)
{
    // ESP_LOGI(TAG, "Sensor client, event %u, addr:0x%04x, RSSI:%d", event, param_1->addr, param->params->ctx.recv_rssi);
    if( param->status_cb.level_status.present_level )
    {
        ESP_LOGI(TAG, "Battery Node = %d (mAh)", param->status_cb.level_status.present_level);
    }

    // switch (event) {
    // case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
    //     ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT");
    //     if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
    //         ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET, onoff %d", param->status_cb.onoff_status.present_onoff);
    //     }
    //     break;
    // case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
    //     ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT");
    //     if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
    //         ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET, onoff %d", param->status_cb.onoff_status.present_onoff);
    //     }
    //     break;
    // case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
    //     ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT");
    //     break;
    // case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
    //     ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT");

    //     break;
    // default:
    //     break;
    // }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_light_client_callback(example_ble_mesh_light_client_cb);
    esp_ble_mesh_register_generic_client_callback(exmaple_ble_mesh_generic_client_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh provisioner");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Sensor client initialized");
    return ESP_OK;
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
    
    mount_SPIFFS();
    xTaskCreate(&led_red_task, "led_red_task", 2048, NULL, 5, NULL);
    xTaskCreate(&led_blue_task, "led_blue_task", 2048, NULL, 5, NULL);
    xTaskCreate(&button_task, "button_task", 2048, NULL, 15, NULL);
    wifi_init();

    err = bluetooth_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

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