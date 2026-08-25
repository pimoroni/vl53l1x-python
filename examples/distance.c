/*
 * distance.c
 *
 * The C equivalent of distance.py: prints the distance read from the sensor,
 * using the "Short Range" timing budget.
 *
 * Build from the repository root with `make examples`, then run
 * ./examples/distance. Press Ctrl+C to exit.
 */

/*
 * usleep() is guarded behind __USE_MISC in glibc's <unistd.h>, which the -std=c99
 * build flag disables. Requesting _DEFAULT_SOURCE re-exposes it. Must be defined
 * before any system header is (transitively) included.
 */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "vl53l1_platform.h"
#include "vl53l1x_python.h"

/* If you're using a software i2c bus (ie: HyperPixel4) then you should
 * `ls /dev/i2c-*` and use the relevant bus number here. Sensors moved with
 * change-address.py answer on their new address. */
#define I2C_BUS "/dev/i2c-1"
#define I2C_ADDRESS 0x29

/* Channel on a TCA9548A I2C multiplexer, or 255 for no multiplexer. */
#define TCA9548A_DEVICE 255
#define TCA9548A_ADDRESS 0

#define REG_MODEL_ID 0x010f
#define MODEL_ID_VL53L1X 0xea

static int i2c_fd = -1;
static volatile sig_atomic_t running = 1;

/*
 * The platform layer routes all bus traffic through the three callbacks below,
 * which VL53L1_set_i2c() installs. The Python module passes ctypes wrappers
 * around smbus2 here; from C we drive /dev/i2c-N directly.
 *
 * Sensor register addresses are 16 bit, and are sent big-endian ahead of the
 * data as part of the same transaction.
 */
static int i2c_read(uint8_t address, uint16_t reg, uint8_t *data, uint8_t length)
{
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xff};
    struct i2c_msg messages[2] = {
        {.addr = address, .flags = 0, .len = sizeof(reg_buf), .buf = reg_buf},
        {.addr = address, .flags = I2C_M_RD, .len = length, .buf = data},
    };
    struct i2c_rdwr_ioctl_data transfer = {.msgs = messages, .nmsgs = 2};

    return ioctl(i2c_fd, I2C_RDWR, &transfer) < 0 ? -1 : 0;
}

static int i2c_write(uint8_t address, uint16_t reg, uint8_t *data, uint8_t length)
{
    /* Two register address bytes, plus the largest payload a uint8_t length
     * can describe. */
    uint8_t buf[2 + 255];
    struct i2c_msg message = {.addr = address, .flags = 0, .len = 2 + length, .buf = buf};
    struct i2c_rdwr_ioctl_data transfer = {.msgs = &message, .nmsgs = 1};

    buf[0] = reg >> 8;
    buf[1] = reg & 0xff;
    memcpy(buf + 2, data, length);

    return ioctl(i2c_fd, I2C_RDWR, &transfer) < 0 ? -1 : 0;
}

/* Selects a channel on the TCA9548A. Only called when a multiplexer is in use. */
static int i2c_multi(uint8_t address, uint16_t channel_mask)
{
    uint8_t value = channel_mask & 0xff;
    struct i2c_msg message = {.addr = address, .flags = 0, .len = 1, .buf = &value};
    struct i2c_rdwr_ioctl_data transfer = {.msgs = &message, .nmsgs = 1};

    return ioctl(i2c_fd, I2C_RDWR, &transfer) < 0 ? -1 : 0;
}

static void handle_interrupt(int signal_number)
{
    (void)signal_number;
    running = 0;
}

int main(void)
{
    VL53L1_DEV dev;
    uint8_t model_id = 0;

    printf("distance.c\n\n"
           "Display the distance read from the sensor.\n\n"
           "Uses the \"Short Range\" timing budget by default.\n\n"
           "Press Ctrl+C to exit.\n\n");

    signal(SIGINT, handle_interrupt);

    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) {
        fprintf(stderr, "Unable to open %s: %s\n", I2C_BUS, strerror(errno));
        return 1;
    }

    /* Must happen before any call into the library, or every bus access fails
     * with "i2c bus read/write not set." */
    VL53L1_set_i2c(i2c_multi, i2c_read, i2c_write);

    if (i2c_read(I2C_ADDRESS, REG_MODEL_ID, &model_id, 1) < 0 || model_id != MODEL_ID_VL53L1X) {
        fprintf(stderr, "VL53L1X not found on address 0x%02x of %s\n", I2C_ADDRESS, I2C_BUS);
        close(i2c_fd);
        return 1;
    }

    dev = initialise(I2C_ADDRESS, TCA9548A_DEVICE, TCA9548A_ADDRESS, 0);
    if (dev == NULL) {
        fprintf(stderr, "Unable to allocate the device\n");
        close(i2c_fd);
        return 1;
    }

    /* An explicit timing budget, in microseconds, paired with an
     * inter-measurement period in milliseconds, replaces the preset the
     * distance mode selects. Set both before ranging starts, then pass a
     * distance mode of 0 to startRanging() to leave the preset alone. */

    /* 1 = Short Range, 2 = Medium Range, 3 = Long Range, 0 = Unchanged. */
    startRanging(dev, 1);

    while (running) {
        /* Blocks until a reading is returned. */
        int32_t distance_in_mm = getDistance(dev);
        printf("Distance: %dmm\n", distance_in_mm);
        fflush(stdout);
        usleep(100000);
    }

    printf("\n");
    stopRanging(dev);
    free(dev);
    close(i2c_fd);

    return 0;
}
