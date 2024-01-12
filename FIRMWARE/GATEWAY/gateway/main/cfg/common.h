/**
 * @file common.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define MESH_MODE 11
#define SMARTCONFIG_MODE 20
#define WIFI_SOFTAP_MODE 31

typedef enum
{
    NOT_STATE,
    POWER_ON_PROVISIONING,
    SMARTCONFIG,
    FOTA,
    PROVISIONING,
    WIFI_SOFTAP,
}status_blue_t;

typedef enum
{
    LOCAL_MODE,
    NORMAL_MODE,
    CONFIG_MODE,
}status_red_t;

typedef struct 
{   
    char cmd[20];
    char action[15];
    int unicast_addr;
    int state;
    char url[50];
    int timeout;
} mqtt_obj_t;

esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj);

#endif