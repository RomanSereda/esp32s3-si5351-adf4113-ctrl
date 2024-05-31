#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "si5351.h"
#include "adf4113.h"
#include "blink_led.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define SYSTEM_CLOCK 10000000

// ADF4113
#define SPI_PORT_NAME SPI2_HOST
#define SPI_PIN_MOSI  35
#define SPI_PIN_CLK	  36
#define SPI_PIN_CS	  39

// SI5351
#define I2C_SDA 8
#define I2C_CLK 9
#define SPI_MAX_TRANSFER_SIZE_BYTES	32	

void setup_si5351_clk()
{
    si5351_init(0, I2C_SDA, I2C_CLK);
    //si5351_setupClk0(100000000, SI5351_DRIVE_STRENGTH_4MA);
    si5351_setupClk2(SYSTEM_CLOCK, SI5351_DRIVE_STRENGTH_8MA);
    si5351_enableOutputs(/*(1<<0) |*/ (1<<2));
	vTaskDelay(1/portTICK_PERIOD_MS);
}

void setup_adf4113_rf()
{
	rfic_config_t rfic_config = {.reference_frec_hz = SYSTEM_CLOCK, 
                                 .resolution_freq_hz = 50e3, 
                                 .vco_center_frec_hz = 2293e6};
	adf4113_init(SPI_PORT_NAME, 
                 SPI_PIN_MOSI, 
                 SPI_PIN_CLK, 
                 SPI_PIN_CS, 
                 SYSTEM_CLOCK, 
                 &rfic_config);

	vTaskDelay(500/portTICK_PERIOD_MS);
}

void app_main(void)
{   
	configure_led();
	setup_si5351_clk();
	setup_adf4113_rf();

    adf4113_set_freq(2286e6);
}
