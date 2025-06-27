/*
 * ina226.c
 *
 * Continuous 1 Hz monitoring of INA226:
 *   – Bus voltage (V), with software calibration
 *   – Shunt voltage (V)
 *   – Current (A), with software calibration
 *   – Power (W), scaled by same current factor
 *
 * Hardware:
 *   • I²C device: /dev/i2c-3
 *   • INA226 address: 0x40 (7-bit)
 *   • Shunt resistor: 0.1 Ω (100 mΩ)
 *   • Expected max current: 3.2 A
 *
 * Calibration data (measured vs. raw):
 *   • Bus voltage: raw 8.30 V → actual 7.99 V ⇒ BUS_V_SCALE ≈ 0.96265
 *   • Current:     raw 0.211 A → actual 0.25 A ⇒ CURRENT_SCALE ≈ 1.18483
 *
 * Build:
 *   gcc -o ina226 ina226.c
 *
 * Run (requires root or i2c access):
 *   sudo ./ina226
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_DEVICE    "/dev/i2c-3"
#define INA226_ADDR   0x40  // 7-bit I2C address

/* INA226 register addresses */
#define REG_CONFIG    0x00
#define REG_SHUNT_V   0x01
#define REG_BUS_V     0x02
#define REG_POWER     0x03
#define REG_CURRENT   0x04
#define REG_CALIB     0x05

/* Hardware parameters */
#define R_SHUNT_OHM      0.1f   // Shunt resistor 0.1 Ω
#define MAX_CURRENT_A    3.2f   // Expected maximum current 3.2 A

/* Software calibration factors (measured/raw) */
#define BUS_V_SCALE      (7.99f/8.30f)   // ≈0.96265
#define CURRENT_SCALE    (0.25f/0.211f)  // ≈1.18483

/* CONFIG register fields */
#define AVG_MODE         5   // average 16 samples
#define VBUS_CT          4   // 1.1 ms conversion time
#define VSH_CT           4   // 1.1 ms conversion time
#define MODE_CONTINUOUS  7   // continuous Shunt+Bus

static volatile sig_atomic_t stop_flag = 0;

/* Signal handler for clean exit on Ctrl+C */
void handle_sigint(int sig) {
    (void)sig;
    stop_flag = 1;
}

/* Write a 16-bit register (big-endian) */
int write_reg(int fd, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };
    return write(fd, buf, 3) == 3 ? 0 : -1;
}

/* Read a 16-bit register (big-endian) */
int read_reg(int fd, uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/**
 * Initialize INA226:
 *   - open I2C device
 *   - select slave address
 *   - write CONFIG register
 *   - calculate and write CALIBRATION register
 * Returns file descriptor or -1 on error.
 */
int ina226_init(const char *device, uint8_t addr7) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Open I2C device");
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, addr7) < 0) {
        perror("Select I2C slave");
        close(fd);
        return -1;
    }

    /* CONFIG: avg=16, VBUSCT=1.1ms, VSHCT=1.1ms, mode=continuous */
    uint16_t cfg = (AVG_MODE << 9)
                 | (VBUS_CT  << 6)
                 | (VSH_CT   << 3)
                 | MODE_CONTINUOUS;
    if (write_reg(fd, REG_CONFIG, cfg) < 0) {
        perror("Write CONFIG");
        close(fd);
        return -1;
    }

    /* CALIBRATION: Current_LSB = MAX_CURRENT_A / 2^15
     *              CAL = trunc(0.00512 / (Current_LSB * R_SHUNT))
     */
    float current_lsb = MAX_CURRENT_A / 32768.0f;
    uint16_t cal = (uint16_t)(0.00512f / (current_lsb * R_SHUNT_OHM));
    if (write_reg(fd, REG_CALIB, cal) < 0) {
        perror("Write CALIBRATION");
        close(fd);
        return -1;
    }

    return fd;
}

int main(void) {
    signal(SIGINT, handle_sigint);

    int fd = ina226_init(I2C_DEVICE, INA226_ADDR);
    if (fd < 0) {
        fprintf(stderr, "Failed to initialize INA226\n");
        return EXIT_FAILURE;
    }

    /* Calculate current LSB for raw→A conversion */
    const float current_lsb = MAX_CURRENT_A / 32768.0f;

    /* Delay for first conversion */
    sleep(1);

    printf("Press Ctrl+C to exit\n");
    while (!stop_flag) {
        uint16_t raw_shunt, raw_bus, raw_curr, raw_power;
        if (read_reg(fd, REG_SHUNT_V, &raw_shunt)   ||
            read_reg(fd, REG_BUS_V,   &raw_bus)     ||
            read_reg(fd, REG_CURRENT, &raw_curr)    ||
            read_reg(fd, REG_POWER,   &raw_power)) {
            fprintf(stderr, "\nI2C read error\n");
            break;
        }

        /* Convert raw readings to physical values */
        float v_shunt = (int16_t)raw_shunt * 2.5e-6f;       // V
        float raw_vbus = raw_bus * 1.25e-3f;               // V
        float v_bus = raw_vbus * BUS_V_SCALE;              // calibrated V

        float raw_i = raw_curr * current_lsb;              // A
        float current = raw_i * CURRENT_SCALE;             // calibrated A

        float power = raw_power * (20.0f * current_lsb)    // W
                      * CURRENT_SCALE;                    // apply same current scale

        /* Print on single line, refresh every second */
        printf("\rShunt: %.6f V | Bus: %.3f V | I: %.3f A | P: %.3f W",
               v_shunt, v_bus, current, power);
        fflush(stdout);

        sleep(1);
    }

    printf("\nExiting...\n");
    close(fd);
    return EXIT_SUCCESS;
}
