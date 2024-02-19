
#include "blink_led.h"
#include "tinyusb_msc_storage.h"

void app_main(void)
{
    configure_led();
    const esp_partition_t* partition = tinyusb_msc_storage();

    blink_led();
}
