/**
 * @file led.c
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

#include "driver/gpio.h"

#include "driver/gpio.h"


#include "wifi/wifi_sta.h"
#include "wifi/wifi_ap.h"
#include "cfg/common.h"
#include "interface/button.h"
#include "interface/led.h"

#include "spiffs/spiffs.h"
#include "web_server/web_server.h"

static const char *TAG = "LED";
extern status_red_t status_red;
extern status_blue_t status_blue;

void led_init(gpio_num_t gpio_num)
{
    gpio_config_t led_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL << gpio_num)};
    gpio_config(&led_cfg);
    gpio_set_level(gpio_num, LED_OFF);
}

void led_red_task(void *param)
{
    led_init(LED_STATUS_RED);
    while (1)
    {
        switch (status_red)
        {
        case LOCAL_MODE:
            gpio_set_level(LED_STATUS_RED, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_RED, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_RED, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_RED, LED_OFF);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
        case NORMAL_MODE:
            gpio_set_level(LED_STATUS_RED, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        case CONFIG_MODE:
            gpio_set_level(LED_STATUS_RED, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        default:
            gpio_set_level(LED_STATUS_RED, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        }
    }
}

void led_blue_task(void *param)
{
    led_init(LED_STATUS_BLUE);
    while (1)
    {
        switch (status_blue)
        {
        case POWER_ON_PROVISIONING:
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
        case SMARTCONFIG:
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        case FOTA:
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            break;
        case PROVISIONING:
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        case WIFI_SOFTAP:
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_ON);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
        case NOT_STATE:
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        default:
            gpio_set_level(LED_STATUS_BLUE, LED_OFF);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        }
    }
}