/**
 * @file wifi_ap.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _WIFI_AP_H_
#define _WIFI_AP_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define WIFI_AP_SSID "Kyuubi"
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONN 2

void wifi_init_softap(void);

#endif