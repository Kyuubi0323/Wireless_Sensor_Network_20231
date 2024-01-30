/**
 * @file led.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _LED_H_
#define _LED_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>


#define LED_ON 1
#define LED_OFF 0
#define LED_STATUS_RED GPIO_NUM_33
#define LED_STATUS_BLUE GPIO_NUM_32

void led_red_task(void *param);
void led_blue_task(void *param);

#endif


