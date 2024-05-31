#include "adf4113.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"

#define ADF4113_PRESCALE8         0
#define ADF4113_PRESCALE16        1
#define ADF4113_PRESCALE32        2
#define ADF4113_PRESCALE64        3

#define ADF4113_PRE8_MIN_N        56
#define ADF4113_PRE16_MIN_N       240
#define ADF4113_PRE32_MIN_N       992
#define ADF4113_PRE64_MIN_N       4032


enum adf4113_control_bits {
    adf4113_r_counter = 0,
    adf4113_n_counter,
    adf4113_function_latch,
    adf4113_init_latch
};

enum adf4113_pd_polarity {
    adf4113_pfd_negative = 0,
    adf4113_pfd_positive
};

enum adf4113_timer_counter_control {
    adf4113_timeout_3cycles = 0,
    adf4113_timeout_7cycles,
    adf4113_timeout_11cycles,
    adf4113_timeout_15cycles,
    adf4113_timeout_19cycles,
    adf4113_timeout_23cycles,
    adf4113_timeout_27cycles,
    adf4113_timeout_31cycles,
    adf4113_timeout_35cycles,
    adf4113_timeout_39cycles,
    adf4113_timeout_43cycles,
    adf4113_timeout_47cycles,
    adf4113_timeout_51cycles,
    adf4113_timeout_55cycles,
    adf4113_timeout_59cycles,
    adf4113_timeout_63cycles
};

enum adf4113_muxout_control {
    adf4113_mux_tristate = 0,
    adf4113_mux_dld,
    adf4113_mux_divn_out,
    adf4113_mux_dvdd,
    adf4113_mux_divr_out,
    adf4113_mux_ald,
    adf4113_mux_serial_data,
    adf4113_mux_dgnd,
};

enum adf4113_resync {
    adf4113_resync_normal1 = 0,
    adf4113_resync_nondelayrf,
    adf4113_resync_normal2,
    adf4113_resync_delayrf
};

enum adf4113_lock_detect_precision {
    adf4113_ld_prec_3cycle = 0,
    adf4113_ld_prec_5cycle
};

enum adf4113_anti_blacklash_width {
    adf4113_anti_width_3p0ns = 0,
    adf4113_anti_width_1p5ns,
    adf4113_anti_width_6p0ns
};

enum adf4113_current_setting {
    adf4113_0p63ma = 0x00,
    adf4113_1p25ma = 0x09,
    adf4113_1p88ma = 0x12,
    adf4113_2p50ma = 0x1b,
    adf4113_3p13ma = 0x24,
    adf4113_3p75ma = 0x2d,
    adf4113_4p38ma = 0x36,
    adf4113_5p00ma = 0x3f,
};

enum adf4113_cp_three_state {
    adf4113_cp_out_normal = 0,
    adf4113_cp_out_tristate
};

enum adf4113_counter_reset {
    adf4113_counter_reset_norm = 0,
    adf4113_counter_reset_held
};

typedef union {
    uint32_t data;
    struct __attribute__((packed)) {
        uint8_t  control_bits : 2;
        uint8_t  counter_reset : 1;
        uint8_t  power_down_1 : 1;
        uint8_t  muxout_control : 3;
        uint8_t  pd_polarity : 1;
        uint8_t  cp_three_state : 1;
        uint8_t  fast_lock_enable : 1;
        uint8_t  fast_lock_mode : 1;
        uint8_t  timer_counter_control : 4;
        uint8_t  current_setting : 6;
        uint8_t  power_down_2 : 1;
        uint8_t  prescaler_value : 2;
    };
} function_latch_t;

typedef union {
    uint32_t data;
    struct __attribute__((packed, aligned(8))) {
        uint8_t  control_bits : 2;
        uint16_t r_counter : 14;
        uint8_t  anti_blacklash_width : 2;
        uint8_t  test_mode_bits : 2;
        uint8_t  lock_detect_precision : 1;
        uint8_t  resunc : 2;
        uint8_t  reserved : 1;
    };
} r_counter_latch_t;

typedef union {
    uint32_t data;
    struct __attribute__((packed, aligned(8))) {
        uint8_t   control_bits : 2;
        uint8_t   a_counter : 6;
        uint16_t  b_counter : 13;
        uint8_t   cp_gain : 1;
        uint8_t   reserved : 2;
    };
} n_counter_latch_t;

typedef struct {
    uint16_t R;
    uint16_t N;
    uint16_t B;
    uint8_t  A;
    uint8_t  prescale;
} counter_param_t;

spi_device_handle_t spi_device_handle;
spi_bus_config_t spi_bus_config;
rfic_config_t adf4113_config;

#define SPI_MAX_TRANSFER_SIZE_BYTES	32

void adf4113_write_bytes(uint8_t* data, int len)
{
	spi_transaction_t spi_transaction;
	memset(&spi_transaction, 0, sizeof(spi_transaction));	
	spi_transaction.length = len * 8;							 
	spi_transaction.tx_buffer = data;						
	ESP_ERROR_CHECK(spi_device_polling_transmit(spi_device_handle, &spi_transaction));
}

void adf4113_write_data(uint32_t func_data, uint32_t r_data, uint32_t n_data)
{
    adf4113_write_bytes((uint8_t*)&func_data, 3); 
    adf4113_write_bytes((uint8_t*)&r_data, 3);    
    adf4113_write_bytes((uint8_t*)&n_data, 3);    
    
    printf("Func: 0x%x\n", (unsigned)func_data);
    printf("Rcnt: 0x%x\n", (unsigned)r_data);
    printf("Ncnt: 0x%x\n", (unsigned)n_data);
}

static counter_param_t calculate_counter_param(uint32_t rf_Freq)
{
    counter_param_t counter_param = {0};

    uint16_t  R, B;
    uint8_t   A, P = 0, p_mode = 0;
    uint16_t  N_val = 0;
    N_val = (rf_Freq / adf4113_config.resolution_freq_hz);
    R = adf4113_config.reference_frec_hz / adf4113_config.resolution_freq_hz;
    
    if( N_val < ADF4113_PRE8_MIN_N) { 
        return counter_param; 
    } else if(( N_val> ADF4113_PRE8_MIN_N) && (N_val < ADF4113_PRE16_MIN_N)) { 
        P = 8;  
        p_mode = ADF4113_PRESCALE8;
    } else if(( N_val > ADF4113_PRE16_MIN_N) && (N_val < ADF4113_PRE32_MIN_N)) { 
        P = 16;
        p_mode = ADF4113_PRESCALE16;
        
    } else if((N_val > ADF4113_PRE32_MIN_N) && ( N_val < ADF4113_PRE64_MIN_N)) { 
        P = 32;
        p_mode = ADF4113_PRESCALE32;
        
    } else if( N_val > ADF4113_PRE64_MIN_N) { 
        P = 64; 
        p_mode = ADF4113_PRESCALE64;
    }
    
    A = N_val%P;
    B = (N_val - A) / P;
    if(B < A) return counter_param;

    counter_param.R = R; 
    counter_param.B = B; 
    counter_param.A = A;  
    counter_param.prescale = (p_mode & 0x03);     // 2-bits
    counter_param.N = N_val;

    return counter_param;
}

void adf4113_init(spi_host_device_t name, uint8_t mosi_pin, uint8_t clk_pin, uint8_t cs_pin, uint32_t clk_value_hz, rfic_config_t* rfic_config)
{
    memcpy(&adf4113_config, rfic_config, sizeof(rfic_config_t));

	spi_bus_config_t config={
		.miso_io_num = -1,
		.mosi_io_num = mosi_pin,
		.sclk_io_num = clk_pin,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = SPI_MAX_TRANSFER_SIZE_BYTES,
		//.intr_flags = ESP_INTR_FLAG_LOWMED,				//<<<Optional, include to ensure SPI doesn't consume a level 1 irq if you are short of irqs
	};

    memcpy(&spi_bus_config, &config, sizeof(spi_bus_config_t));
	ESP_ERROR_CHECK(spi_bus_initialize(name, &spi_bus_config, SPI_DMA_CH_AUTO));		//<<<You can use ESP_ERROR_CHECK() on this call if you are having issues

	spi_device_interface_config_t spi_device_interface_config = {
		.clock_speed_hz = SPI_MASTER_FREQ_8M,		//Clock out 
		.mode = 0,									//<<< SPI mode 0
		.spics_io_num = cs_pin,			            //<<< CS pin number
		.queue_size = 1,							//<<< Number of transactions we want to be able to queue at a time using spi_device_queue_trans()
        .flags = SPI_DEVICE_3WIRE
	};
	ESP_ERROR_CHECK(spi_bus_add_device(name, &spi_device_interface_config, &spi_device_handle));

    counter_param_t counter_param = calculate_counter_param(adf4113_config.vco_center_frec_hz);

    function_latch_t function_latch = {0};
    function_latch.prescaler_value = counter_param.prescale;
    function_latch.pd_polarity     = adf4113_pfd_positive;
    function_latch.muxout_control  = 0;
    function_latch.control_bits    = adf4113_init_latch;

    r_counter_latch_t r_counter_latch = {0};
    r_counter_latch.r_counter = counter_param.R;
    r_counter_latch.control_bits = adf4113_r_counter;

    n_counter_latch_t n_counter_latch = {0};
    n_counter_latch.a_counter = counter_param.A;
    n_counter_latch.b_counter = counter_param.B;
    n_counter_latch.control_bits = adf4113_n_counter;

    adf4113_write_data(function_latch.data, 
                       r_counter_latch.data, 
                       n_counter_latch.data);
}

bool adf4113_set_freq(uint32_t frec_hz)
{
    counter_param_t counter_param = calculate_counter_param(frec_hz);

    function_latch_t function_latch = {0};
    function_latch.prescaler_value = counter_param.prescale;
    function_latch.pd_polarity     = adf4113_pfd_positive;
    function_latch.muxout_control  = adf4113_mux_divn_out;
    function_latch.control_bits    = adf4113_function_latch;
    function_latch.current_setting = adf4113_5p00ma;

    r_counter_latch_t r_counter_latch = {0};
    r_counter_latch.r_counter = counter_param.R;
    r_counter_latch.control_bits = adf4113_r_counter;

    n_counter_latch_t n_counter_latch = {0};
    n_counter_latch.a_counter = counter_param.A;
    n_counter_latch.b_counter = counter_param.B;
    n_counter_latch.control_bits = adf4113_n_counter;

    adf4113_write_data(function_latch.data, 
                       r_counter_latch.data, 
                       n_counter_latch.data);

    return true;
}
