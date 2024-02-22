set(priv_requires fatfs console)

if(CONFIG_ESP32_STORAGE_MEDIA_SPIFLASH)
    list(APPEND priv_requires wear_levelling esp_partition)
endif()

idf_component_register(
    SRCS "main.c" "tinyusb_msc_storage.c" "blink_led.c" "si5351.c"
    INCLUDE_DIRS .
    PRIV_REQUIRES "${priv_requires}"
)
