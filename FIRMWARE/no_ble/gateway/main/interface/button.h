/**
 * @file button.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#define BUTTON_CONFIG_PIN GPIO_NUM_23
#define TIME_HOLD (3000 / portTICK_PERIOD_MS)
#define TIME_CLICK_MIN (20 / portTICK_PERIOD_MS)
#define TIME_CLICK_MAX (1000 / portTICK_PERIOD_MS)
#define TIME_RESET (1000 / portTICK_PERIOD_MS)

#define BUTTON_TRIGGER 0
#define BUTTON_NOT_TRIGGER 1

void button_task(void *param);

typedef struct
{
    uint32_t time_down;
    uint8_t pin;
    uint32_t time_up;
    uint32_t deltaT;
    uint8_t click_cnt;
    uint32_t time_stamp;
} button_t;

#endif

