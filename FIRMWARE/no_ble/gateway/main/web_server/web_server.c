/**
 * @file web_server.c
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

#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_http_client.h"
#include "esp_attr.h"
#include "mqtt_client.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"

#include "driver/gpio.h"

#include "wifi/wifi_sta.h"
#include "wifi/wifi_ap.h"
#include "cfg/common.h"
#include "interface/button.h"
#include "interface/led.h"


#include "spiffs/spiffs.h"
#include "web_server/web_server.h"

static const char *TAG = "WEBSERVER";
extern httpd_handle_t server;
extern RingbufHandle_t webserver_ring_buf;

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

esp_err_t get_smart_gateway_server_handler(httpd_req_t *req)
{
    extern const unsigned char index_html_start[] asm("_binary_webserver_html_start");
    extern const unsigned char index_html_end[] asm("_binary_webserver_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_size);
    return ESP_OK;
}

esp_err_t post_wifi_handler(httpd_req_t *req)
{
    char data_recv[100] = {0};
    httpd_req_recv(req, data_recv, req->content_len);
    UBaseType_t ret = xRingbufferSend(webserver_ring_buf, data_recv, req->content_len, portMAX_DELAY);
    if (ret != pdTRUE)
        ESP_LOGE(TAG, "Failed to send item\n");
    return ESP_OK;
}
esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[] asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}
httpd_uri_t get_smart_gateway_server = {
    .uri = "/server",
    .method = HTTP_GET,
    .handler = get_smart_gateway_server_handler,
    .user_ctx = NULL,
};

httpd_uri_t post_wifi = {
    .uri = "/post_wifi",
    .method = HTTP_POST,
    .handler = post_wifi_handler,
    .user_ctx = NULL,
};
httpd_uri_t icon_favi = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = favicon_get_handler,
    .user_ctx = NULL,
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &get_smart_gateway_server);
        httpd_register_uri_handler(server, &post_wifi);
        httpd_register_uri_handler(server, &icon_favi);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
        return server;
    }
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

void disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

void connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}