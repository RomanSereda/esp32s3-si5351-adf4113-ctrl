#pragma once
#include "stdint.h"
#include "stdbool.h"
#include "driver/spi_master.h"

typedef struct {
    uint32_t vco_center_frec_hz;
    uint32_t reference_frec_hz;
    uint32_t resolution_freq_hz;
} rfic_config_t;

void adf4113_init(spi_host_device_t name, 
                  uint8_t mosi_pin, 
                  uint8_t clk_pin, 
                  uint8_t cs_pin, 
                  uint32_t clk_value_hz, 
                  rfic_config_t* adf4113_config);

bool adf4113_set_freq(uint32_t frec_hz);
