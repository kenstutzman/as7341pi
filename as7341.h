#ifndef AS7341_H
#define AS7341_H

#include <stdint.h>
#include <stdbool.h>

// AS7341 register definitions
#define AS7341_WHOAMI           0x92
#define AS7341_CHIP_ID          0x09
#define AS7341_ENABLE           0x80
#define AS7341_CFG6             0xAF
#define AS7341_CH0_DATA_L       0x95
//#define AS7341_STATUS2          0x94  // wrong register number!
#define AS7341_STATUS2          0xA3
#define AS7341_FD_STATUS        0xDB

// Enums
typedef enum {
    AS7341_ADC_CHANNEL_0,
    AS7341_ADC_CHANNEL_1,
    AS7341_ADC_CHANNEL_2,
    AS7341_ADC_CHANNEL_3,
    AS7341_ADC_CHANNEL_4,
    AS7341_ADC_CHANNEL_5
} as7341_adc_channel_t;

typedef enum {
    AS7341_COLOR_CHANNEL_F1,
    AS7341_COLOR_CHANNEL_F2,
    AS7341_COLOR_CHANNEL_F3,
    AS7341_COLOR_CHANNEL_F4,
    AS7341_COLOR_CHANNEL_F5,
    AS7341_COLOR_CHANNEL_F6,
    AS7341_COLOR_CHANNEL_F7,
    AS7341_COLOR_CHANNEL_F8,
    AS7341_COLOR_CHANNEL_CLEAR,
    AS7341_COLOR_CHANNEL_NIR
} as7341_color_channel_t;

typedef enum {
    AS7341_WAITING_START,
    AS7341_WAITING_LOW,
    AS7341_WAITING_HIGH,
    AS7341_WAITING_DONE
} as7341_reading_state_t;

typedef enum {
    AS7341_SMUX_CMD_ROM_RESET = 0,
    AS7341_SMUX_CMD_READ = 1,
    AS7341_SMUX_CMD_WRITE = 2
} as7341_smux_cmd_t;

// Function prototypes
bool as7341_begin(uint8_t i2c_bus, uint8_t i2c_addr, int32_t sensor_id);
void as7341_end(void);
void as7341_power_enable(bool enable);
uint16_t as7341_read_channel(as7341_adc_channel_t channel);
uint16_t as7341_get_channel(as7341_color_channel_t channel);
bool as7341_read_all_channels(uint16_t *readings_buffer);
bool as7341_start_reading(void);
bool as7341_check_reading_progress(void);
bool as7341_get_all_channels(uint16_t *readings_buffer);
void as7341_delay_for_data(int wait_time);
void as7341_set_smux_low_channels(bool f1_f4);
bool as7341_enable_spectral_measurement(bool enable);
bool as7341_enable_smux(void);
bool as7341_get_is_data_ready(void);
void as7341_setup_f1f4_clear_nir(void);
void as7341_setup_f5f8_clear_nir(void);
bool as7341_set_smux_command(as7341_smux_cmd_t command);

#endif // AS7341_H
