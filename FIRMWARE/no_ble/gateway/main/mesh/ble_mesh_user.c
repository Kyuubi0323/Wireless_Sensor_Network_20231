/**
 * @file ble_mesh_user.c
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

#define ONOFF_GROUP_ADDR 0xC000
#define HEARTBEAT_NODE_GROUP_ADDR 0xC001
#define HEARTBEAT_GATEWAY_GROUP_ADDR 0xC005
static const char *TAG = "BLE MESH USER";
extern TaskHandle_t prov_dev_handle;
extern char topic_commands_set[50];
extern char topic_commands_get[50];
extern char topic_commands_status[50];
extern char topic_commands_heartbeat_node[50];
extern char topic_commands_heartbeat_gateway[50];
extern char topic_commands_network[50];
extern char topic_commands_process[50];
extern char topic_commands_version[50];
extern char topic_commands_fota[50];
extern char topic_commands_group[50];
extern char topic_commands_provision[50];
extern esp_mqtt_client_handle_t client;
extern status_red_t status_red;
extern status_blue_t status_blue;
extern uint8_t dev_uuid[16];

TimerHandle_t xTimer;
TimerHandle_t hb_node_timer;
char last_uuid_nw[160] = {0};

static struct esp_ble_mesh_key
{
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t app_key[16];
} prov_key;

node_info_t nodes[MAXIMUM_NODE] = {
    [0 ... MAXIMUM_NODE - 1] = {
        .unicast_node = ESP_BLE_MESH_ADDR_UNASSIGNED,
        .elem_num = 0,
    },
};

esp_ble_mesh_client_t config_client;
esp_ble_mesh_client_t onoff_client;

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static const esp_ble_mesh_client_op_pair_t vnd_op_pair[] = {
    {ESP_BLE_MESH_VND_MODEL_OP_SEND, ESP_BLE_MESH_VND_MODEL_OP_STATUS},
    {ESP_BLE_MESH_VND_MODEL_OP_HB, ESP_BLE_MESH_VND_MODEL_OP_STATUS},
};

static esp_ble_mesh_client_t vendor_client = {
    .op_pair_size = ARRAY_SIZE(vnd_op_pair),
    .op_pair = vnd_op_pair,
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 0),
    ESP_BLE_MESH_MODEL_OP_END,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_client_pub, 2 + 3, ROLE_PROVISIONER);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_client_pub, &onoff_client),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT, vnd_op, NULL, &vendor_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid = dev_uuid,
    .prov_unicast_addr = PROV_OWN_ADDR,
    .prov_start_address = 0x0005,
    .prov_attention = 0x00,
    .prov_algorithm = 0x00,
    .prov_pub_key_oob = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags = 0x00,
    .iv_index = 0x00,
};

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid)
{
    if (dev_uuid == NULL)
    {
        ESP_LOGE(TAG, "%s, Invalid device uuid", __func__);
        return;
    }

    /* Copy device address to the device uuid with offset equals to 2 here.
     * The first two bytes is used for matching device uuid by Provisioner.
     * And using device address here is to avoid using the same device uuid
     * by different unprovisioned devices.
     */
    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);
}

void vTimerCallback(TimerHandle_t xTimer)
{
    if (status_blue == POWER_ON_PROVISIONING)
    {
        status_blue = NOT_STATE;
    }
    ESP_LOGW(TAG, "Stop provisioning");
    xTimerStop(xTimer, 0);
    xTimerDelete(xTimer, 0);
}

void hb_node_timer_cb(TimerHandle_t hb_node_timer)
{
    if (status_blue != FOTA)
        ble_mesh_send_vendor_message_all(0x00, ESP_BLE_MESH_VND_MODEL_OP_HB, false);
}

esp_err_t bluetooth_init(void)
{
    esp_err_t ret;
    xTimer = xTimerCreate("Timer", 30000 / portTICK_PERIOD_MS, pdFALSE, (void *)0, vTimerCallback);
    xTimerStart(xTimer, 0);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return ret;
    }

    ble_mesh_read_node_info_in_flash(UUID_NETWORK_FILE, last_uuid_nw);
    ESP_LOGW(TAG, "Read node info in flash: %s", bt_hex(last_uuid_nw, strlen(last_uuid_nw)));
    return ret;
}

esp_err_t ble_mesh_delete_node_info_in_flash(char *file_name)
{
    remove_file(file_name);
    return ESP_OK;
}

bool ble_mesh_read_node_info_in_flash(char *file_name, char *uuid_network)
{
    read_from_file(file_name, uuid_network);
    if (strlen(uuid_network) > 0)
        return true;
    return false;
}

void ble_mesh_store_node_info_in_flash(char *file_name)
{
    int count = 0;
    char *uuid_network = NULL;
    ble_mesh_delete_node_info_in_flash(file_name);
    for (int index = 0; index < ARRAY_SIZE(nodes); index++)
    {
        if (strlen((char *)nodes[index].uuid) > 0)
        {
            count++;
        }
    }
    uuid_network = (char *)malloc((count + 1) * 16 + 1);
    memset(uuid_network, '\0', (count + 1) * 16 + 1);
    for (int index = 0; index < ARRAY_SIZE(nodes); index++)
    {
        if (strlen((char *)nodes[index].uuid) > 0)
        {
            strcat(uuid_network, (char *)nodes[index].uuid);
        }
    }
    write_to_file(file_name, uuid_network);
    free(uuid_network);
}

void decode_comp_data(node_info_t *comp, uint8_t *comp_data, int elem_num)
{
    int pos = 0;
    comp->cid = (uint16_t)(comp_data[1] << 8 | comp_data[0]);
    comp->pid = (uint16_t)(comp_data[3] << 8 | comp_data[2]);
    comp->vid = (uint16_t)(comp_data[5] << 8 | comp_data[4]);
    comp->crpl = (uint16_t)(comp_data[7] << 8 | comp_data[6]);
    comp->feature = (uint16_t)(comp_data[9] << 8 | comp_data[8]);
    pos = 10;
    for (int i = 0; i < elem_num; i++)
    {
        int j = 0;
        int k = 0;
        comp->elem[i].loc = (uint16_t)(comp_data[pos + 1] << 8 | comp_data[pos]);
        pos += 2;
        comp->elem[i].numS = comp_data[pos++];
        comp->elem[i].numV = comp_data[pos++];
        for (j = 0; j < comp->elem[i].numS; j++)
        {
            comp->elem[i].models[j].model_id = (uint16_t)(comp_data[pos + 1] << 8 | comp_data[pos]);
            pos += 2;
            if (comp->elem[i].models[j].model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV)
            {
                strcpy((char *)comp->elem[i].models[j].model_name, "onoff_sv");
                comp->elem[i].models[j].onoff_state = OFF;
            }
        }
        for (k = 0; k < comp->elem[i].numV; k++)
        {
            comp->elem[i].models[j + k].company_id = (uint16_t)(comp_data[pos + 1] << 8 | comp_data[pos]);
            comp->elem[i].models[j + k].model_id = (uint16_t)(comp_data[pos + 3] << 8 | comp_data[pos + 2]);
            pos += 4;
        }
    }
}

esp_err_t ble_mesh_store_node_info(const uint8_t uuid[16], uint16_t unicast, uint8_t elem_num)
{
    if (!uuid || !ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (!memcmp(nodes[i].uuid, uuid, 16))
        {
            ESP_LOGW(TAG, "%s: reprovisioned device 0x%04x", __func__, unicast);
            nodes[i].unicast_node = unicast;
            nodes[i].elem_num = elem_num;
            for (int j = 0; j < elem_num; j++)
            {
                nodes[i].elem[j].unicast_elem = unicast + j;
            }
            return ESP_OK;
        }
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (nodes[i].unicast_node == ESP_BLE_MESH_ADDR_UNASSIGNED)
        {
            memcpy(nodes[i].uuid, uuid, 16);
            nodes[i].unicast_node = unicast;
            nodes[i].elem_num = elem_num;
            for (int j = 0; j < elem_num; j++)
            {
                nodes[i].elem[j].unicast_elem = unicast + j;
            }
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

void ble_mesh_store_composition_info(node_info_t *comp, uint8_t *comp_data, int elem_num)
{
    decode_comp_data(comp, comp_data, elem_num);
}

node_info_t *ble_mesh_get_node_info_with_unicast(uint16_t unicast)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        ESP_LOGE(TAG, "Error unicast address: 0x%04x", unicast);
        return NULL;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (nodes[i].unicast_node <= unicast && nodes[i].unicast_node + nodes[i].elem_num > unicast)
        {
            return &nodes[i];
        }
    }
    ESP_LOGE(TAG, "Not found node with unicast address: 0x%04x", unicast);
    return NULL;
}

node_info_t *ble_mesh_get_node_info_with_uuid(uint8_t *uuid)
{
    if (!uuid || strlen((char *)uuid) != 16)
    {
        ESP_LOGE(TAG, "Error uuid");
        return NULL;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (strstr((char *)nodes[i].uuid, (char *)uuid) != NULL)
        {
            return &nodes[i];
        }
    }
    ESP_LOGE(TAG, "Not found node with uuid: %s", bt_hex(uuid, 16));
    return NULL;
}

elem_info_t *ble_mesh_get_elem_info_with_unicast(uint16_t unicast)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        ESP_LOGE(TAG, "Error unicast address: 0x%04x", unicast);
        return NULL;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (nodes[i].unicast_node <= unicast && nodes[i].unicast_node + nodes[i].elem_num > unicast)
        {
            for (int j = 0; j < ARRAY_SIZE(nodes[i].elem); j++)
            {
                if (nodes[i].elem[j].unicast_elem == unicast)
                {
                    return &nodes[i].elem[j];
                }
            }
        }
    }
    return NULL;
}

model_info_t *ble_mesh_get_model_info_with_model_id(uint16_t unicast, uint16_t model_id)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        ESP_LOGE(TAG, "Error unicast address: 0x%04x", unicast);
        return NULL;
    }

    elem_info_t *elem = NULL;
    elem = ble_mesh_get_elem_info_with_unicast(unicast);
    for (int i = 0; i < ARRAY_SIZE(elem->models); i++)
    {
        if (elem->models[i].model_id == model_id)
        {
            return &elem->models[i];
        }
    }
    return NULL;
}

void ble_mesh_send_vendor_message_timeout(esp_ble_mesh_msg_ctx_t *ctx, uint8_t state, uint32_t opcode)
{
    esp_err_t err = esp_ble_mesh_client_model_send_msg(vendor_client.model, ctx, opcode, sizeof(state), &state, MSG_TIMEOUT, true, MSG_ROLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send vendor message 0x%06x", opcode);
        return;
    }
}

void ble_mesh_send_vendor_message(node_info_t *node, uint8_t state, uint32_t opcode, bool need_rsp)
{
    esp_ble_mesh_msg_ctx_t ctx = {0};
    esp_err_t err;
    ctx.net_idx = prov_key.net_idx;
    ctx.app_idx = prov_key.app_idx;
    ctx.addr = node->unicast_node;
    ctx.send_ttl = MSG_SEND_TTL;
    ctx.send_rel = MSG_SEND_REL;

    err = esp_ble_mesh_client_model_send_msg(vendor_client.model, &ctx, opcode, sizeof(state), &state, MSG_TIMEOUT, need_rsp, MSG_ROLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send vendor message 0x%06x", opcode);
        return;
    }
}

void ble_mesh_send_vendor_message_all(uint8_t state, uint32_t opcode, bool need_rsp)
{
    esp_ble_mesh_msg_ctx_t ctx = {0};
    esp_err_t err;
    ctx.net_idx = prov_key.net_idx;
    ctx.app_idx = prov_key.app_idx;
    ctx.addr = 0xFFFF;
    ctx.send_ttl = MSG_SEND_TTL;
    ctx.send_rel = MSG_SEND_REL;

    err = esp_ble_mesh_client_model_send_msg(vendor_client.model, &ctx, opcode, sizeof(state), &state, MSG_TIMEOUT, need_rsp, MSG_ROLE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send vendor message 0x%06x", opcode);
        return;
    }
}

esp_err_t ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem,
                                  esp_ble_mesh_model_t *client_model, uint32_t opcode)
{
    if (!common || !client_model)
    {
        return ESP_ERR_INVALID_ARG;
    }

    common->opcode = opcode;
    common->model = client_model;
    common->ctx.net_idx = prov_key.net_idx;
    common->ctx.app_idx = prov_key.app_idx;
    common->ctx.addr = unicast_elem;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->ctx.send_rel = MSG_SEND_REL;
    common->msg_timeout = MSG_TIMEOUT;
    common->msg_role = MSG_ROLE;

    return ESP_OK;
}

esp_err_t ble_mesh_onoff_get_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model)
{
    esp_ble_mesh_generic_client_get_state_t get_state = {0};
    ble_mesh_set_msg_common(common, unicast_elem, onoff_client_model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET);
    esp_err_t err = esp_ble_mesh_generic_client_get_state(common, &get_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Generic OnOff Get failed", __func__);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_onoff_set_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model, uint8_t state)
{
    esp_ble_mesh_generic_client_set_state_t set_state = {0};
    model_info_t *model = ble_mesh_get_model_info_with_model_id(unicast_elem, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
    model->target_state = state;
    ble_mesh_set_msg_common(common, unicast_elem, onoff_client_model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET);
    set_state.onoff_set.op_en = false;
    set_state.onoff_set.onoff = state;
    set_state.onoff_set.tid = 0;
    esp_err_t err = esp_ble_mesh_generic_client_set_state(common, &set_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Generic OnOff Set failed", __func__);
    }
    return ESP_OK;
}

esp_err_t ble_mesh_config_model_app_bind(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
    set_state.model_app_bind.element_addr = unicast_elem;
    set_state.model_app_bind.model_app_idx = prov_key.app_idx;
    set_state.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
    set_state.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Config Model App Bind failed", __func__);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_config_sub_add(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD);
    set_state.model_sub_add.element_addr = unicast_elem;
    set_state.model_sub_add.sub_addr = HEARTBEAT_GATEWAY_GROUP_ADDR;
    set_state.model_sub_add.model_id = ESP_BLE_MESH_VND_MODEL_ID_SERVER;
    set_state.model_sub_add.company_id = CID_ESP;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Model Subscription Add");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_config_heartbeat_pub_set(esp_ble_mesh_client_common_param_t *common, node_info_t *node, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_SET);
    set_state.heartbeat_pub_set.dst = HEARTBEAT_NODE_GROUP_ADDR;
    set_state.heartbeat_pub_set.count = 0xFF;
    set_state.heartbeat_pub_set.period = 0x05;
    set_state.heartbeat_pub_set.ttl = 5;
    set_state.heartbeat_pub_set.feature = 0x0001;
    set_state.heartbeat_pub_set.net_idx = prov_key.net_idx;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Heartbeat Publication Set");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_config_heartbeat_sub_set(esp_ble_mesh_client_common_param_t *common, node_info_t *node, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_HEARTBEAT_SUB_SET);
    set_state.heartbeat_sub_set.src = PROV_OWN_ADDR;
    set_state.heartbeat_sub_set.dst = node->unicast_node;
    set_state.heartbeat_sub_set.period = 0x05;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Heartbeat Publication Set");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_config_pub_set(esp_ble_mesh_client_common_param_t *common, node_info_t *node, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET);
    set_state.model_pub_set.element_addr = node->unicast_node;
    set_state.model_pub_set.publish_addr = ONOFF_GROUP_ADDR;
    set_state.model_pub_set.publish_app_idx = prov_key.app_idx;
    set_state.model_pub_set.cred_flag = false;
    set_state.model_pub_set.publish_ttl = 0xFF;
    set_state.model_pub_set.publish_period = 0x00;
    set_state.model_pub_set.publish_retransmit = 0x61;
    set_state.model_pub_set.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
    set_state.model_pub_set.company_id = ESP_BLE_MESH_CID_NVAL;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Heartbeat Publication Set");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_config_node_reset(esp_ble_mesh_client_common_param_t *common, node_info_t *node, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_NODE_RESET);
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Heartbeat Publication Set");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_deinit(void)
{
    esp_err_t err = ESP_OK;
    esp_ble_mesh_deinit_param_t param;
    param.erase_flash = false;
    err = esp_ble_mesh_deinit(&param);
    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "esp_ble_mesh_deinit success (err %d)", err);
    }
    else
    {
        ESP_LOGE(TAG, "esp_ble_mesh_deinit fail (err %d)", err);
    }
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    return err;
}
static esp_err_t prov_complete(int node_idx, const esp_ble_mesh_octet16_t uuid,
                               uint16_t unicast, uint8_t elem_num, uint16_t net_idx)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};
    node_info_t *node = NULL;
    char name[11] = {0};
    esp_err_t err;

    // ESP_LOGI(TAG, "node index: 0x%x, unicast address: 0x%02x, element num: %d, netkey index: 0x%02x",
    //          node_idx, unicast, elem_num, net_idx);
    // ESP_LOGI(TAG, "device uuid: %s", bt_hex(uuid, 16));

    sprintf(name, "%s%d", "NODE-", node_idx);
    err = esp_ble_mesh_provisioner_set_node_name(node_idx, name);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Set node name failed", __func__);
        return ESP_FAIL;
    }

    err = ble_mesh_store_node_info(uuid, unicast, elem_num);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Store node info failed", __func__);
        return ESP_FAIL;
    }

    node = ble_mesh_get_node_info_with_unicast(unicast);
    if (!node)
    {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return ESP_FAIL;
    }

    ble_mesh_store_node_info_in_flash(UUID_NETWORK_FILE);

    ESP_LOGW(TAG, "********************** Node Info **********************");
    ESP_LOGW(TAG, "Node index: 0x%02x, Unicast address: 0x%04x, Element num: %d, Netkey index: 0x%02x",
             node_idx, node->unicast_node, node->elem_num, net_idx);
    ESP_LOGW(TAG, "Device UUID: %s", bt_hex(uuid, 16));
    ESP_LOGW(TAG, "*******************************************************");
    ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void prov_link_open(esp_ble_mesh_prov_bearer_t bearer)
{
    ESP_LOGI(TAG, "%s link open", bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
}

static void prov_link_close(esp_ble_mesh_prov_bearer_t bearer, uint8_t reason)
{
    ESP_LOGI(TAG, "%s link close, reason 0x%02x",
             bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", reason);
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};

    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner will only
     * use this callback to report the devices, whose device UUID starts with 0xdd & 0xdd,
     * to the application layer.
     */
    // ESP_LOGI(TAG, "address: %s, address type: %d, adv type: %d", bt_hex(addr, BD_ADDR_LEN), addr_type, adv_type);
    // ESP_LOGI(TAG, "device uuid: %s", bt_hex(dev_uuid, 16));
    // ESP_LOGI(TAG, "oob info: %d, bearer: %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    /* Note: If unprovisioned device adv packets have not been received, we should not add
             device with ADD_DEV_START_PROV_NOW_FLAG set. */
    if (status_blue == PROVISIONING)
    {
        memcpy(add_dev.addr, addr, BD_ADDR_LEN);
        add_dev.addr_type = (uint8_t)addr_type;
        memcpy(add_dev.uuid, dev_uuid, 16);
        add_dev.oob_info = oob_info;
        add_dev.bearer = (uint8_t)bearer;
        esp_ble_mesh_provisioner_add_unprov_dev(&add_dev, ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG);
    }
    else if (status_blue == POWER_ON_PROVISIONING)
    {
        if (strstr(bt_hex(last_uuid_nw, strlen(last_uuid_nw)), bt_hex(dev_uuid, 8)) != NULL)
        {
            memcpy(add_dev.addr, addr, BD_ADDR_LEN);
            add_dev.addr_type = (uint8_t)addr_type;
            memcpy(add_dev.uuid, dev_uuid, 16);
            add_dev.oob_info = oob_info;
            add_dev.bearer = (uint8_t)bearer;
            esp_ble_mesh_provisioner_add_unprov_dev(&add_dev, ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG);
        }
    }
    // ESP_LOGW(TAG, "Free heap size: %d", esp_get_free_heap_size());
    // ESP_LOGW(TAG, "Minimum free heap size: %d", esp_get_minimum_free_heap_size());
    return;
}

static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                     esp_ble_mesh_prov_cb_param_t *param)
{
    // ESP_LOGW(TAG, "%s, event = 0x%02x", __func__, event);
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        prov_link_open(param->provisioner_prov_link_open.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        prov_link_close(param->provisioner_prov_link_close.bearer, param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                      param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                      param->provisioner_prov_complete.netkey_idx);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code %d", param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
    {
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d", param->provisioner_set_node_name_comp.err_code);
        if (param->provisioner_set_node_name_comp.err_code == ESP_OK)
        {
            const char *name = NULL;
            name = esp_ble_mesh_provisioner_get_node_name(param->provisioner_set_node_name_comp.node_index);
            if (!name)
            {
                ESP_LOGE(TAG, "Get node name failed");
                return;
            }
            // ESP_LOGI(TAG, "Node %d name is: %s", param->provisioner_set_node_name_comp.node_index, name);
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
    {
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == ESP_OK)
        {
            esp_err_t err = 0;
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Provisioner bind local model appkey failed");
                return;
            }
            err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx, ESP_BLE_MESH_VND_MODEL_ID_CLIENT, CID_ESP);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Provisioner bind local model appkey failed");
                return;
            }
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_ENABLE_HEARTBEAT_RECV_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ENABLE_HEARTBEAT_RECV_COMP_EVT, err_code %d", param->provisioner_enable_heartbeat_recv_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_HEARTBEAT_MESSAGE_EVT:
    {
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_HEARTBEAT_MESSAGE_EVT");
        ESP_LOGI(TAG, "hb_src: 0x%04x, hb_dst: 0x%04x, init_ttl: 0x%02x, rx_ttl: 0x%02x", param->provisioner_recv_heartbeat.hb_src, param->provisioner_recv_heartbeat.hb_dst, param->provisioner_recv_heartbeat.init_ttl, param->provisioner_recv_heartbeat.rx_ttl);
        ESP_LOGI(TAG, "hops: 0x%02x, feature: 0x%04x, rssi: %d", param->provisioner_recv_heartbeat.hops, param->provisioner_recv_heartbeat.feature, (int)param->provisioner_recv_heartbeat.rssi);
        if (status_red == NORMAL_MODE)
        {
            char hb_payload[200] = {0};
            node_info_t *node = ble_mesh_get_node_info_with_unicast(param->provisioner_recv_heartbeat.hb_src);
            sprintf(hb_payload, "{\"action\":\"hb_node\",\"uuid\":\"%s\",\"unicast_addr\":%d,\"rssi\":%d}", bt_hex(node->uuid, 8), (int)node->unicast_node, (int)param->provisioner_recv_heartbeat.rssi);
            esp_mqtt_client_publish(client, topic_commands_heartbeat_node, hb_payload, strlen(hb_payload), 0, 0);
        }
        break;
    }
    case ESP_BLE_MESH_MODEL_SUBSCRIBE_GROUP_ADDR_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_SUBSCRIBE_GROUP_ADDR_COMP_EVT");
        break;
    default:
        break;
    }
    return;
}

static void ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                      esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t addr;
    int err;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    ESP_LOGW(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
             __func__, param->error_code, event, param->params->ctx.addr, opcode);

    if (param->error_code)
    {
        ESP_LOGE(TAG, "Send config client message failed, opcode 0x%04x", opcode);
        return;
    }

    node = ble_mesh_get_node_info_with_unicast(addr);
    if (!node)
    {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event)
    {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: 0x%04x", addr);
            ESP_LOGW(TAG, "********************** Composition Data **********************");
            // ESP_LOGI(TAG, "Composition data %s", bt_hex(param->status_cb.comp_data_status.composition_data->data, param->status_cb.comp_data_status.composition_data->len));
            ble_mesh_store_composition_info(node, param->status_cb.comp_data_status.composition_data->data, node->elem_num);
            ESP_LOGW(TAG, "CID: 0x%04x, PID: 0x%04x, VID: 0x%04x, CRPL: 0x%04x, Feature: 0x%04x",
                     node->cid, node->pid, node->vid, node->crpl, node->feature);
            for (uint8_t i = 0; i < node->elem_num; i++)
            {
                uint8_t j = 0;
                ESP_LOGW(TAG, "ELEMENT %u", i);
                ESP_LOGW(TAG, "      Unicast: 0x%04x, Location: 0x%04x, NumS: %u, NumV: %u",
                         node->elem[i].unicast_elem, node->elem[i].loc, node->elem[i].numS, node->elem[i].numV);
                for (j = 0; j < node->elem[i].numS; j++)
                {
                    ESP_LOGW(TAG, "            MODEL %u, ID: 0x%04x", j, node->elem[i].models[j].model_id);
                }
                for (uint8_t k = j; k < node->elem[i].numV + j; k++)
                {
                    ESP_LOGW(TAG, "            MODEL %u, ID: 0x%04x", k, node->elem[i].models[k].model_id);
                }
            }
            ESP_LOGW(TAG, "**************************************************************");
            esp_ble_mesh_cfg_client_set_state_t set_state = {0};
            ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set_state.app_key_add.net_idx = prov_key.net_idx;
            set_state.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err)
            {
                ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                return;
            }
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET: 0x%04x", addr);
            ble_mesh_config_heartbeat_pub_set(&common, node, config_client.model);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        {
            // char group_payload[100] = {0};
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD: 0x%04x", addr);
            // sprintf(group_payload, "{\"action\":\"add_group\",\"group_addr\":\"0x%04x\",\"unicast_addr\":%d}", ONOFF_GROUP_ADDR, addr);
            // if (status == NORMAL_MODE)
            //     esp_mqtt_client_publish(client, topic_commands_group, group_payload, strlen(group_payload), 0, 0);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: 0x%04x", addr);
            ble_mesh_config_model_app_bind(&common, node, node->unicast_node, config_client.model);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        {
            char nw_payload[200] = {0};
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: 0x%04x", addr);
            // ble_mesh_config_sub_add(&common, node, node->unicast_node, config_client.model);
            ble_mesh_config_pub_set(&common, node, config_client.model);
            sprintf(nw_payload, "{\"action\":\"join\",\"uuid\":\"%s\",\"unicast_addr\":%d,\"type\":\"switch\",\"switch_num\":%d}", bt_hex(node->uuid, 8), node->unicast_node, node->elem_num);
            if (status_red == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_network, nw_payload, strlen(nw_payload), 0, 0);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_SET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_SET: 0x%04x", addr);
            // ble_mesh_config_heartbeat_sub_set(&common, node, config_client.model);
            // ble_mesh_config_sub_add(&common, node, node->unicast_node, config_client.model);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_HEARTBEAT_SUB_SET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_HEARTBEAT_SUB_SET: 0x%04x", addr);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_NODE_RESET:
        {
            // esp_ble_mesh_provisioner_delete_node_with_addr(node->unicast_node);
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_NODE_RESET: 0x%04x", addr);
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS:
            ESP_LOG_BUFFER_HEX("composition data %s", param->status_cb.comp_data_status.composition_data->data,
                               param->status_cb.comp_data_status.composition_data->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_STATUS:
            break;
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET:
        {
            ESP_LOGW(TAG, "Get composition data timeout, unicast address: 0x%04x", addr);
            esp_ble_mesh_cfg_client_get_state_t get_state = {0};
            ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get_state.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get_state);
            if (err)
            {
                ESP_LOGE(TAG, "%s: Config Composition Data Get failed", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        {
            ESP_LOGW(TAG, "Model add app key timeout, unicast address: 0x%04x", addr);
            esp_ble_mesh_cfg_client_set_state_t set_state = {0};
            ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set_state.app_key_add.net_idx = prov_key.net_idx;
            set_state.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err)
            {
                ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        {
            ble_mesh_config_model_app_bind(&common, node, node->unicast_node, config_client.model);
            ESP_LOGW(TAG, "Model App Bind timeout, unicast address: 0x%04x", addr);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        {
            ble_mesh_config_sub_add(&common, node, node->unicast_node, config_client.model);
            ESP_LOGW(TAG, "Model Sub Add timeout, unicast address: 0x%04x", addr);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_HEARTBEAT_PUB_SET:
        {
            ble_mesh_config_heartbeat_pub_set(&common, node, config_client.model);
            ESP_LOGW(TAG, "Model Heartbeat Publication Set timeout, unicast address: 0x%04x", addr);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_HEARTBEAT_SUB_SET:
        {
            ble_mesh_config_heartbeat_sub_set(&common, node, config_client.model);
            ESP_LOGW(TAG, "Model Heartbeat Subscription Set timeout, unicast address: 0x%04x", addr);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_NODE_RESET:
        {
            // ble_mesh_config_node_reset(&common, node, config_client.model);
            ESP_LOGW(TAG, "Config Node Reset timeout, unicast address: 0x%04x", addr);
            break;
        }
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                       esp_ble_mesh_generic_client_cb_param_t *param)
{
    node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t addr;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    ESP_LOGW(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
             __func__, param->error_code, event, param->params->ctx.addr, opcode);

    if (param->error_code)
    {
        ESP_LOGE(TAG, "Send generic client message failed, opcode 0x%04x", opcode);
        return;
    }

    node = ble_mesh_get_node_info_with_unicast(addr);
    if (!node)
    {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event)
    {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        {
            model_info_t *onoff_model = NULL;
            char get_payload[200] = {0};
            onoff_model = ble_mesh_get_model_info_with_model_id(param->params->ctx.addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            if (!onoff_model)
            {
                ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                return;
            }
            onoff_model->onoff_state = param->status_cb.onoff_status.present_onoff;
            sprintf(get_payload, "{\"action\":\"onoff\",\"method\":\"get\",\"unicast_addr\":%u,\"state\":%u}", addr, onoff_model->onoff_state);
            if (status_red == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, get_payload, strlen(get_payload), 0, 0);
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET unicast: 0x%04x, onoff: 0x%02x", addr, onoff_model->onoff_state);
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        {
            char set_payload[200] = {0};
            model_info_t *onoff_model = ble_mesh_get_model_info_with_model_id(addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            if (!onoff_model)
            {
                ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                return;
            }
            onoff_model->onoff_state = param->status_cb.onoff_status.present_onoff;
            sprintf(set_payload, "{\"action\":\"onoff\",\"method\":\"set\",\"unicast_addr\":%u,\"state\":%u}", addr, onoff_model->onoff_state);
            if (status_red == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, set_payload, strlen(set_payload), 0, 0);
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET unicast: 0x%04x, onoff: 0x%02x", addr, onoff_model->onoff_state);
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS:
        {
            char status_payload[200] = {0};
            for (int index = 0; index < node->elem_num; index++)
            {
                model_info_t *onoff_model = ble_mesh_get_model_info_with_model_id(addr + index, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
                if (!onoff_model)
                {
                    ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                    return;
                }
                onoff_model->onoff_state = (param->status_cb.onoff_status.present_onoff >> index) & 0x01;
                sprintf(status_payload, "{\"action\":\"onoff\",\"method\":\"status\",\"unicast_addr\":%u,\"state\":%u}", addr + index, onoff_model->onoff_state);
                if (status_red == NORMAL_MODE)
                    esp_mqtt_client_publish(client, topic_commands_status, status_payload, strlen(status_payload), 0, 0);
            }
            ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT STATUS, unicast: 0x%04x, state: 0x%02x", addr, param->status_cb.onoff_status.present_onoff);
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        /* If failed to receive the responses, these messages will be resend */
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET TIMEOUT, unicast: 0x%04x", addr);
            esp_ble_mesh_client_common_param_t common;
            ble_mesh_onoff_get_state(&common, addr, onoff_client.model);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET TIMEOUT, unicast: 0x%04x", addr);
            esp_ble_mesh_client_common_param_t common;
            model_info_t *model = ble_mesh_get_model_info_with_model_id(addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            ble_mesh_onoff_set_state(&common, addr, onoff_client.model, model->target_state);
            break;
        }
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
    ESP_LOGW(TAG, "%s: event 0x%02x", __func__, event);
    switch (event)
    {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OPERATION_EVT");
        ESP_LOGW(TAG, "opcode: 0x%06x, msg: 0x%02x", param->model_operation.opcode, *(param->model_operation.msg));
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_SEND_COMP_EVT");
        if (param->model_send_comp.err_code)
            ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT");
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT");
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
    {
        uint8_t state;
        ESP_LOGI(TAG, "ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT");
        ESP_LOGI(TAG, "opcode: 0x%06x", param->client_send_timeout.opcode);
        if (param->client_send_timeout.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND)
        {
            state = 0x02;
            ble_mesh_send_vendor_message_timeout(param->client_send_timeout.ctx, state, param->client_send_timeout.opcode);
        }
        else if (param->client_send_timeout.opcode == ESP_BLE_MESH_VND_MODEL_OP_HB)
        {
            state = 0x01;
            ble_mesh_send_vendor_message_timeout(param->client_send_timeout.ctx, state, param->client_send_timeout.opcode);
        }
        break;
    }
    case ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT");
        break;
    case ESP_BLE_MESH_SERVER_MODEL_UPDATE_STATE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_SERVER_MODEL_UPDATE_STATE_COMP_EVT");
        ESP_LOGW(TAG, "result %d, model id 0x%04x, type 0x%02x",
                 param->server_model_update_state.err_code,
                 param->server_model_update_state.model->model_id,
                 param->server_model_update_state.type);
        break;
    case ESP_BLE_MESH_MODEL_EVT_MAX:
        ESP_LOGE(TAG, "ESP_BLE_MESH_MODEL_EVT_MAX");
        break;
    default:
        break;
    }
}

esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = {0xdd, 0xdd};
    esp_err_t err = ESP_OK;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(ble_mesh_generic_client_cb);
    esp_ble_mesh_register_custom_model_callback(ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_client_model_init(&vnd_models[0]);
    if (err)
    {
        ESP_LOGE(TAG, "Failed to initialize vendor client");
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set matching device uuid (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh provisioner (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add local AppKey (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_recv_heartbeat(true);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable heartbeat");
        return err;
    }

    err = esp_ble_mesh_model_subscribe_group_addr(esp_ble_mesh_get_primary_element_address(), BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ONOFF_GROUP_ADDR);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to subscribe group address");
    }

    hb_node_timer = xTimerCreate("Heartbeat Node", 10000 / portTICK_PERIOD_MS, pdTRUE, (void *)0, hb_node_timer_cb);
    xTimerStart(hb_node_timer, 0);

    ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return err;
}