idf_component_register(SRCS "app_main.c"
                            "cjson/cJSON.c"
                            "mqtt/mqtt.c"
                            "smartcfg/smartcfg.c"
                            "wifi/wifi_ap.c"
                            "wifi/wifi_sta.c"
               INCLUDE_DIRS "."
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
