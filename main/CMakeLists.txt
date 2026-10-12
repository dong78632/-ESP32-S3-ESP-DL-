# Local components/esp-dl takes priority over the managed Git dependency.
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        nvs_flash
        esp_system
        esp_timer
        esp_pm
        esp_wifi
        esp_event
        esp_netif
        mqtt
        json
        esp32-camera
        esp-dl
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
