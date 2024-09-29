/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 0
// 1       @
// 2
// 3             @
// 4
// 5
// 6
// 7
//
// E.g. if slave addresses 0x12 and 0x34 were acknowledged.

#include <stdint.h>
#include <stdio.h>

#include "bmp3.h"
#include "common.h"
#include "pico/stdlib.h"
#include "sys_fn.h"
#include "sys_i2c.h"

#define I2C_PORT i2c0

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) { return (addr & 0x78) == 0 || (addr & 0x78) == 0x78; }

int main() {
    // Init system enviroment
    sys_init();

    // init i2c0 with default pins and 100kHz
    sys_i2c_init_def(I2C_PORT, 100000, true);

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = sys_i2c_rbyte(i2c0, addr, &rxdata);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    int8_t rslt;
    uint8_t loop = 0;
    uint16_t settings_sel;
    struct bmp3_dev dev;
    struct bmp3_data data = {0};
    struct bmp3_settings settings = {0};
    struct bmp3_status status = {{0}};

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(I2C_PORT, &dev, BMP3_I2C_INTF);
#ifdef BMP_ALL
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel =
        BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    while (loop < 100) {
        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            /* NOTE : Read status register again to clear data ready interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

#ifdef BMP3_FLOAT_COMPENSATION
            printf("Data[%d]  T: %.2f deg C, P: %.2f Pa\n", loop, (data.temperature), (data.pressure));
#else
            printf("Data[%d]  T: %ld deg C, P: %lu Pa\n", loop, (long int)(int32_t)(data.temperature / 100),
                   (long unsigned int)(uint32_t)(data.pressure / 100));
#endif

            loop = loop + 1;
        }
    }

    bmp3_coines_deinit();
#endif
    return rslt;

    printf("Done.\n");
    return 0;
}