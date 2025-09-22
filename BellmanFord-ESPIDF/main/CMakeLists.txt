idf_component_register(
    SRCS
        "main.cpp"
        "LoRaRouting.cpp"
        "node.cpp"
        "lora_sx1276.cpp"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        esp_driver_spi      # driver/spi_master.h
        esp_driver_gpio     # driver/gpio.h
        esp_timer           # esp_timer.h
        esp_hw_support      # esp_mac.h (esp_read_mac)
        esp_system          # esp_random.h
        log                 # esp_log.h
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
