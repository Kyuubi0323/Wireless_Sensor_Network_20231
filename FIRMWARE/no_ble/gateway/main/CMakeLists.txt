idf_component_register(SRCS "app_main.c"
                            "cjson/cJSON_Utils.c"
                            "cjson/cJSON.c"
                            "mqtt/mqtt.c"
                            "smartcfg/smartcfg.c"
                            "wifi/wifi_ap.c"
                            "wifi/wifi_sta.c"
                            "interface/button.c"
                            "interface/led.c"
                            "web_server/web_server.c"
                            "spiffs/spiffs.c"
                            "cfg/common.c"
                            "fota/fota.c"
               INCLUDE_DIRS "."
               EMBED_FILES "favicon.ico"
               EMBED_TXTFILES ${project_dir}/certs/git_ota.crt ${project_dir}/html/webserver.html
               REQUIRES esp_event esp_http_server esp_http_client esp_https_ota esp_wifi esp_partition mqtt 
                       			freertos spiffs nvs_flash driver vfs)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
