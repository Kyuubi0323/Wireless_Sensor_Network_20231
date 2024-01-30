/**
 * @file mqtt.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _MQTT_H_
#define _MQTT_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_USERNAME "device_01"
#define MQTT_ID "device_01"

// #define MQTT_BROKER "mqtt.thingsboard.cloud"

// #define MQTT_USERNAME "device01"
// #define MQTT_PASSWORD "device01"
// #define MQTT_TOKEN "zzcrb6epswmwu0oygjg1"


void mqtt_client_start(void);

#endif