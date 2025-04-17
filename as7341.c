#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include "as7341.h"

// I2C file descriptor
static int i2c_fd = -1;

// Sensor state
typedef struct {
    uint8_t i2c_addr;
    uint16_t channel_readings[12]; // Buffer for F1-F8, Clear, NIR, etc.
    as7341_reading_state_t reading_state;
    uint8_t last_spectral_int_source;
} as7341_t;

// Static instance of the sensor
static as7341_t sensor;

// Helper function to write a byte to a register
static bool as7341_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (write(i2c_fd, buf, 2) != 2) {
        fprintf(stderr, "Failed to write to register 0x%02X: %s\n", reg, strerror(errno));
        return false;
    }
    return true;
}

// Helper function to read a byte from a register
static bool as7341_read_reg(uint8_t reg, uint8_t *value) {
    if (write(i2c_fd, &reg, 1) != 1) {
        fprintf(stderr, "Failed to set register 0x%02X: %s\n", reg, strerror(errno));
        return false;
    }
    if (read(i2c_fd, value, 1) != 1) {
        fprintf(stderr, "Failed to read from register 0x%02X: %s\n", reg, strerror(errno));
        return false;
    }
    return true;
}

// Helper function to read a 16-bit value from a register (LSB first)
static bool as7341_read_reg16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    if (write(i2c_fd, &reg, 1) != 1) {
        fprintf(stderr, "Failed to set register 0x%02X: %s\n", reg, strerror(errno));
        return false;
    }
    if (read(i2c_fd, buf, 2) != 2) {
        fprintf(stderr, "Failed to read 16-bit from register 0x%02X: %s\n", reg, strerror(errno));
        return false;
    }
    *value = (uint16_t)(buf[0] | (buf[1] << 8)); // LSB first
    return true;
}

// Initialize the AS7341 sensor
bool as7341_begin(uint8_t i2c_bus, uint8_t i2c_addr, int32_t sensor_id) {
    char i2c_path[20];

    snprintf(i2c_path, sizeof(i2c_path), "/dev/i2c-%d", i2c_bus);

    // Open I2C bus
    i2c_fd = open(i2c_path, O_RDWR);
    if (i2c_fd < 0) {
        fprintf(stderr, "Failed to open I2C bus %s: %s\n", i2c_path, strerror(errno));
        return false;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, i2c_addr) < 0) {
        fprintf(stderr, "Failed to set I2C address 0x%02X: %s\n", i2c_addr, strerror(errno));
        close(i2c_fd);
        return false;
    }

    sensor.i2c_addr = i2c_addr;

    // Verify chip ID
    uint8_t chip_id;
    if (!as7341_read_reg(AS7341_WHOAMI, &chip_id)) {
        close(i2c_fd);
        return false;
    }
    if ((chip_id & 0xFC) != (AS7341_CHIP_ID << 2)) {
        fprintf(stderr, "Invalid chip ID: 0x%02X\n", chip_id);
        close(i2c_fd);
        return false;
    }
    // Enable power
    as7341_power_enable(true);
    return true;
}

// Clean up resources
void as7341_end(void) {
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

// Enable or disable sensor power
void as7341_power_enable(bool enable) {
    uint8_t enable_val;
    as7341_read_reg(AS7341_ENABLE, &enable_val);
    enable_val = (enable_val & ~0x01) | (enable ? 0x01 : 0x00);
    as7341_write_reg(AS7341_ENABLE, enable_val);
}

// Read ADC data for a specific channel
uint16_t as7341_read_channel(as7341_adc_channel_t channel) {
    uint16_t value;
    as7341_read_reg16(AS7341_CH0_DATA_L + 2 * channel, &value);
    return value;
}

// Get stored channel reading
uint16_t as7341_get_channel(as7341_color_channel_t channel) {
    return sensor.channel_readings[channel];
}

// Read all channels (F1-F8, Clear, NIR)
bool as7341_read_all_channels(uint16_t *readings_buffer) {
    // Configure SMUX for low channels (F1-F4, Clear, NIR)
    as7341_set_smux_low_channels(true);
    as7341_enable_spectral_measurement(true);
    as7341_delay_for_data(0);


    // Read low channels
    uint8_t reg = AS7341_CH0_DATA_L;
    if (write(i2c_fd, &reg, 1) != 1) return false;
    uint8_t buf[12];
    if (read(i2c_fd, buf, 12) != 12) return false;
    for (int i = 0; i < 6; i++) {
        readings_buffer[i] = (uint16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));
    }

    // Configure SMUX for high channels (F5-F8, Clear, NIR)
    as7341_set_smux_low_channels(false);
    as7341_enable_spectral_measurement(true);
    as7341_delay_for_data(0);

    // Read high channels
    if (write(i2c_fd, &reg, 1) != 1) return false;
    if (read(i2c_fd, buf, 12) != 12) return false;
    for (int i = 0; i < 6; i++) {
        readings_buffer[6 + i] = (uint16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));
    }

    return true;
}

// Start non-blocking reading
bool as7341_start_reading(void) {
    sensor.reading_state = AS7341_WAITING_START;
    as7341_check_reading_progress();
    return true;
}

// Check reading progress
bool as7341_check_reading_progress(void) {
    if (sensor.reading_state == AS7341_WAITING_START) {
        as7341_set_smux_low_channels(true);
        as7341_enable_spectral_measurement(true);
        sensor.reading_state = AS7341_WAITING_LOW;
        return false;
    }

    if (!as7341_get_is_data_ready() || sensor.reading_state == AS7341_WAITING_DONE) {
        return false;
    }

    if (sensor.reading_state == AS7341_WAITING_LOW) {
        uint8_t reg = AS7341_CH0_DATA_L;
        uint8_t buf[12];
        if (write(i2c_fd, &reg, 1) != 1 || read(i2c_fd, buf, 12) != 12) {
            return false;
        }
        for (int i = 0; i < 6; i++) {
            sensor.channel_readings[i] = (uint16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));
        }

        as7341_set_smux_low_channels(false);
        as7341_enable_spectral_measurement(true);
        sensor.reading_state = AS7341_WAITING_HIGH;
        return false;
    }

    if (sensor.reading_state == AS7341_WAITING_HIGH) {
        sensor.reading_state = AS7341_WAITING_DONE;
        uint8_t reg = AS7341_CH0_DATA_L;
        uint8_t buf[12];
        if (write(i2c_fd, &reg, 1) != 1 || read(i2c_fd, buf, 12) != 12) {
            return false;
        }
        for (int i = 0; i < 6; i++) {
            sensor.channel_readings[6 + i] = (uint16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));
        }
        return true;
    }

    return false;
}

// Get all channels from internal buffer
bool as7341_get_all_channels(uint16_t *readings_buffer) {
    for (int i = 0; i < 12; i++) {
        readings_buffer[i] = sensor.channel_readings[i];
    }
    return true;
}

// Delay for data readiness
void as7341_delay_for_data(int wait_time) {
    if (wait_time == 0) {
        while (!as7341_get_is_data_ready()) {
            usleep(1000); // 1ms
        }
    } else if (wait_time > 0) {
        struct timespec start, current;
        clock_gettime(CLOCK_MONOTONIC, &start);
        long elapsed_ms = 0;
        while (!as7341_get_is_data_ready() && elapsed_ms < wait_time) {
            usleep(1000);
            clock_gettime(CLOCK_MONOTONIC, &current);
            elapsed_ms = (current.tv_sec - start.tv_sec) * 1000 +
                         (current.tv_nsec - start.tv_nsec) / 1000000;
        }
    }
}

// Configure SMUX for low or high channels
void as7341_set_smux_low_channels(bool f1_f4) {
    as7341_enable_spectral_measurement(false);
    as7341_set_smux_command(AS7341_SMUX_CMD_WRITE);
    if (f1_f4) {
        as7341_setup_f1f4_clear_nir();
    } else {
        as7341_setup_f5f8_clear_nir();
    }
    as7341_enable_smux();
}

// Enable spectral measurement
bool as7341_enable_spectral_measurement(bool enable) {
    uint8_t enable_val;
    as7341_read_reg(AS7341_ENABLE, &enable_val);
    enable_val = (enable_val & ~0x02) | (enable ? 0x02 : 0x00);
    return as7341_write_reg(AS7341_ENABLE, enable_val);
}

// Enable SMUX
bool as7341_enable_smux(void) {
    uint8_t enable_val;
    as7341_read_reg(AS7341_ENABLE, &enable_val);
    enable_val |= 0x10; // Set SMUX enable bit
    if (!as7341_write_reg(AS7341_ENABLE, enable_val)) return false;

    int timeout = 1000;
    int count = 0;
    while (count < timeout) {
        as7341_read_reg(AS7341_ENABLE, &enable_val);
        if (!(enable_val & 0x10)) return true;
        usleep(1000);
        count++;
    }
    return false;
}

// Check if data is ready
bool as7341_get_is_data_ready(void) {
    uint8_t status;
    as7341_read_reg(AS7341_STATUS2, &status);
    return (status & 0x40) != 0; // AVALID bit
}

// SMUX configurations
void as7341_setup_f1f4_clear_nir(void) {
    as7341_write_reg(0x00, 0x30); // F3 left to ADC2
    as7341_write_reg(0x01, 0x01); // F1 left to ADC0
    as7341_write_reg(0x02, 0x00); // Reserved
    as7341_write_reg(0x03, 0x00); // F8 left disabled
    as7341_write_reg(0x04, 0x00); // F6 left disabled
    as7341_write_reg(0x05, 0x42); // F4 left to ADC3, F2 left to ADC1
    as7341_write_reg(0x06, 0x00); // F5 left disabled
    as7341_write_reg(0x07, 0x00); // F7 left disabled
    as7341_write_reg(0x08, 0x50); // CLEAR to ADC4
    as7341_write_reg(0x09, 0x00); // F5 right disabled
    as7341_write_reg(0x0A, 0x00); // F7 right disabled
    as7341_write_reg(0x0B, 0x00); // Reserved
    as7341_write_reg(0x0C, 0x20); // F2 right to ADC1
    as7341_write_reg(0x0D, 0x04); // F4 right to ADC3
    as7341_write_reg(0x0E, 0x00); // F6/F8 right disabled
    as7341_write_reg(0x0F, 0x30); // F3 right to ADC2
    as7341_write_reg(0x10, 0x01); // F1 right to ADC0
    as7341_write_reg(0x11, 0x50); // CLEAR right to ADC4
    as7341_write_reg(0x12, 0x00); // Reserved
    as7341_write_reg(0x13, 0x06); // NIR to ADC5
}

void as7341_setup_f5f8_clear_nir(void) {
    as7341_write_reg(0x00, 0x00); // F3 left disabled
    as7341_write_reg(0x01, 0x00); // F1 left disabled
    as7341_write_reg(0x02, 0x00); // Reserved
    as7341_write_reg(0x03, 0x40); // F8 left to ADC3
    as7341_write_reg(0x04, 0x02); // F6 left to ADC1
    as7341_write_reg(0x05, 0x00); // F4/F2 disabled
    as7341_write_reg(0x06, 0x10); // F5 left to ADC0
    as7341_write_reg(0x07, 0x03); // F7 left to ADC2
    as7341_write_reg(0x08, 0x50); // CLEAR to ADC4
    as7341_write_reg(0x09, 0x10); // F5 right to ADC0
    as7341_write_reg(0x0A, 0x03); // F7 right to ADC2
    as7341_write_reg(0x0B, 0x00); // Reserved
    as7341_write_reg(0x0C, 0x00); // F2 right disabled
    as7341_write_reg(0x0D, 0x00); // F4 right disabled
    as7341_write_reg(0x0E, 0x24); // F8 right to ADC2, F6 right to ADC1
    as7341_write_reg(0x0F, 0x00); // F3 right disabled
    as7341_write_reg(0x10, 0x00); // F1 right disabled
    as7341_write_reg(0x11, 0x50); // CLEAR right to ADC4
    as7341_write_reg(0x12, 0x00); // Reserved
    as7341_write_reg(0x13, 0x06); // NIR to ADC5
}

// Set SMUX command
bool as7341_set_smux_command(as7341_smux_cmd_t command) {
    uint8_t cfg6_val;
    as7341_read_reg(AS7341_CFG6, &cfg6_val);
    cfg6_val = (cfg6_val & ~0x18) | (command << 3);
    return as7341_write_reg(AS7341_CFG6, cfg6_val);
}
