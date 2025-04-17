// Copyright (c) 2019 Bryan Siepert for Adafruit Industries
// All rights reserved.

// Modified arduino library to work with raspberry pi
// Ken Stutzman
// 4/16/2025

#include <stdio.h>
#include "as7341.h"

int main(void) {
    printf("F1,F2,F3,F4,F5,F6,F7,F8,F9,F10,Clear,NIR\r\n");
    // Initialize sensor on I2C bus 1, address 0x39
    if (!as7341_begin(1, 0x39, 0)) {
        fprintf(stderr, "Failed to initialize AS7341\n");
        return 1;
    }

    // Read all channels
    uint16_t readings[12];
    if (as7341_read_all_channels(readings)) {
        for (int i = 0; i < 10; i++) {
            printf("%u,",readings[i]);
        }
        printf("%u,",readings[8]); // clear
        printf("%u\r\n",readings[9]);  // nir
    } else {
        fprintf(stderr, "Failed to read channels\n");
    }

    // Clean up
  as7341_end();
  return 0;
}

