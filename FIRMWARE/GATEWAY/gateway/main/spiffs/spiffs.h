/**
 * @file spiffs.h
 * @author Kyuubi0323 (khoi.nv202647@sis.hust.edu.vn)
 * @brief 
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _SPIFFS_H_
#define _SPIFFS_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

void mount_SPIFFS(void);
esp_err_t write_to_file(char *file_name, char *buf);
esp_err_t read_from_file(char *file_name, char *buf);
void remove_file(char *file_name);

#endif