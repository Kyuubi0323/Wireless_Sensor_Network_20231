/**
 * @file ble_mesh_user.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BLE_MESH_USER_H_
#define _BLE_MESH_USER_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define MAXIMUM_NODE 20
#define CID_ESP 0x02E5
#define PROV_OWN_ADDR 0x0001
#define MSG_SEND_TTL 3
#define MSG_SEND_REL false
#define MSG_TIMEOUT 0
#define MSG_ROLE ROLE_PROVISIONER
#define COMP_DATA_PAGE_0 0x00
#define APP_KEY_IDX 0x0000
#define APP_KEY_OCTET 0x12
#define ON 0x1
#define OFF 0x0
#define UNPROV_DEV_QUEUE_SIZE 10;
#define UUID_NETWORK_FILE "uuid_network.txt"
#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_HB        ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

typedef struct
{
    uint16_t company_id;
    uint16_t model_id;
    uint8_t model_name[20];
    uint8_t onoff_state;
    uint8_t target_state;
} model_info_t;

typedef struct
{
    uint16_t unicast_elem;
    uint16_t loc;
    uint8_t numS;
    uint8_t numV;
    model_info_t models[10];
} elem_info_t;

typedef struct
{
    uint8_t uuid[16];
    uint16_t unicast_node;
    uint8_t elem_num;
    uint16_t cid;
    uint16_t pid;
    uint16_t vid;
    uint16_t crpl;
    uint16_t feature;
    elem_info_t elem[10];
} node_info_t;

esp_err_t bluetooth_init(void);
void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);
esp_err_t ble_mesh_init(void);
esp_err_t ble_mesh_config_model_app_bind(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model);
esp_err_t ble_mesh_onoff_set_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model, uint8_t state);
esp_err_t ble_mesh_onoff_get_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model);
esp_err_t ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem,
                                  esp_ble_mesh_model_t *client_model, uint32_t opcode);
model_info_t *ble_mesh_get_model_info_with_model_id(uint16_t unicast, uint16_t model_id);
esp_err_t ble_mesh_config_sub_add(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model);
elem_info_t *ble_mesh_get_elem_info_with_unicast(uint16_t unicast);
node_info_t *ble_mesh_get_node_info_with_uuid(uint8_t *uuid);
node_info_t *ble_mesh_get_node_info_with_unicast(uint16_t unicast);
void ble_mesh_store_composition_info(node_info_t *comp, uint8_t *comp_data, int elem_num);
esp_err_t ble_mesh_store_node_info(const uint8_t uuid[16], uint16_t unicast, uint8_t elem_num);
void decode_comp_data(node_info_t *comp, uint8_t *comp_data, int elem_num);
esp_err_t ble_mesh_delete_node_info_in_flash(char *file_name);
bool ble_mesh_read_node_info_in_flash(char *file_name, char *uuid_network);
void ble_mesh_store_node_info_in_flash(char *file_name);
esp_err_t ble_mesh_deinit(void);
void hb_node_timer_cb(TimerHandle_t hb_node_timer);
void ble_mesh_send_vendor_message(node_info_t *node, uint8_t state, uint32_t opcode, bool need_rsp);
void ble_mesh_send_vendor_message_timeout(esp_ble_mesh_msg_ctx_t *ctx, uint8_t state, uint32_t opcode);
void ble_mesh_send_vendor_message_all(uint8_t state, uint32_t opcode, bool need_rsp);
esp_err_t ble_mesh_config_node_reset(esp_ble_mesh_client_common_param_t *common, node_info_t *node, esp_ble_mesh_model_t *model);
#endif

