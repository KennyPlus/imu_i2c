#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include "bmi270.h"
#include "bmi270_config_file.h"

#define UPDATE_RATE 200.0
#define UPDATE_TIME_NS ((long)(1e9 / UPDATE_RATE))

extern const size_t bmi270_config_file_len; // <- 添加 extern 声明

typedef struct {
    int16_t acc[3];
    int16_t gyr[3];
} imu_data_t;

volatile sig_atomic_t running = 1;

void sig_handler(int signum) {
    running = 0;
}

int main() {
    signal(SIGINT, sig_handler);

    struct bmi270 sensor = {.i2c_addr = I2C_PRIM_ADDR};
    if (bmi270_init(&sensor) != 0) {
        fprintf(stderr, "Sensor initialization failed!\n");
        return -1;
    }

    uint8_t status_reg = read_register(&sensor, 0x21);
    printf("\033[1;33mSTATUS_REG (0x21): 0x%02X\033[0m\n", status_reg);
    printf("\033[1;36mConfig file size: %zu bytes\033[0m\n", bmi270_config_file_len);

    if (load_config_file(&sensor) != 0) {
        fprintf(stderr, "\033[1;31mFeature configuration loading failed!\033[0m\n");
        return -1;
    } else {
        printf("\033[1;32mFeature configuration loaded successfully!\033[0m\n");
    }

    set_mode(&sensor, PERFORMANCE_MODE);
    set_acc_range(&sensor, ACC_RANGE_2G);
    set_gyr_range(&sensor, GYR_RANGE_1000);
    set_acc_odr(&sensor, ACC_ODR_200);
    set_gyr_odr(&sensor, GYR_ODR_200);
    enable_data_streaming(&sensor);
    usleep(5000);

    // 显式设置 Feature Page = 0x00
    uint8_t feat_page = 0x00;
    write_register_block(&sensor, 0x5F, 1, &feat_page);
    usleep(5000);

    // Enable features: Tap, Orientation, Activity, Freefall
    uint8_t feature_enable[2] = {0x7B, 0x00}; // bits: 0x01:ori, 0x08:tap, 0x10:activity, 0x20:ff, 0x40:3tap
    write_register_block(&sensor, 0x50, 2, feature_enable);
    usleep(10000);

    // Trigger feature engine to latch
    uint8_t cmd = 0x00;
    write_register(&sensor, 0x7E, cmd);
    usleep(10000);

    // INT event map
    uint8_t int_map_feat[2] = {0x7B, 0x00};
    write_register_block(&sensor, 0x58, 2, int_map_feat);
    usleep(10000);
    printf("\033[1;34mFeatures enabled and INT mapping configured.\033[0m\n");

    // Tap configuration: full set
    uint8_t tap_cfg_full[] = {
        0x09, // 0x63: TAP_TH
        0x0A, // 0x64: TAP_SHOCK
        0x0F, // 0x65: TAP_QUIET
        0x20  // 0x66: TAP_DURATION
    };
    write_register_block(&sensor, 0x63, sizeof(tap_cfg_full), tap_cfg_full);
    usleep(10000);

    // Orientation configuration
    uint8_t orient_cfg[2] = {0x01, 0x00};
    write_register_block(&sensor, 0x55, 2, orient_cfg);
    usleep(10000);

    // Diagnostic: Check feature enable status
    uint8_t feat_en_status[2] = {0};
    read_register_block(&sensor, 0x53, feat_en_status, 2);
    printf("\033[1;35mFeature Enable Status: 0x%02X 0x%02X\033[0m\n", feat_en_status[0], feat_en_status[1]);

    // Diagnostic: Sensor time
    uint8_t sens_time[3] = {0};
    read_register_block(&sensor, 0x02, sens_time, 3);
    uint32_t time_val = sens_time[0] | (sens_time[1] << 8) | (sens_time[2] << 16);
    printf("\033[1;35mSensor Time: %u\033[0m\n", time_val);

    // Diagnostic: PMU status
    uint8_t pmu_status = read_register(&sensor, 0x03);
    printf("\033[1;35mPMU Status (0x03): 0x%02X\033[0m\n", pmu_status);

    // Diagnostic: Sensor Enable status
    uint8_t sensor_en = read_register(&sensor, 0x40);
    printf("\033[1;35mSensor Enable (0x40): 0x%02X\033[0m\n", sensor_en);

    imu_data_t data;
    struct timespec sleep_time = {0, UPDATE_TIME_NS};

    const double acc_scale = 1.0 / 16384.0;
    const double gyr_scale = 1.0 / 32.8;

    const char* orientation_str[] = {
        "Portrait Upright",
        "Portrait Upside Down",
        "Landscape Left",
        "Landscape Right",
        "Face Up",
        "Face Down",
        "Reserved",
        "Reserved"
    };

    while (running) {
        get_acc_raw(&sensor, &data.acc[0], &data.acc[1], &data.acc[2]);
        get_gyr_raw(&sensor, &data.gyr[0], &data.gyr[1], &data.gyr[2]);

        double acc_g[3], gyr_dps[3];

        for (int i = 0; i < 3; ++i) {
            acc_g[i] = data.acc[i] * acc_scale;
            gyr_dps[i] = data.gyr[i] * gyr_scale;
        }

        printf("\r\033[2KAcc[g]: [Ax=%.3f, Ay=%.3f, Az=%.3f] | Gyr[dps]: [Gx=%.2f, Gy=%.2f, Gz=%.2f]",
               acc_g[0], acc_g[1], acc_g[2],
               gyr_dps[0], gyr_dps[1], gyr_dps[2]);
        fflush(stdout);

        uint8_t int_status0 = read_register(&sensor, 0x1C);
        uint8_t int_status2 = read_register(&sensor, 0x1E);

     //   printf("\n\033[2mINT_STATUS0: 0x%02X | INT_STATUS2: 0x%02X\033[0m\n", int_status0, int_status2);

        if (int_status0 & 0x01) {
            printf(" Single Tap Detected!\n");
        }
        if (int_status0 & 0x10) {
            printf(" Double Tap Detected!\n");
        }
        if (int_status0 & 0x40) {
            printf(" Triple Tap Detected!\n");
        }
        if (int_status0 & 0x20) {
            printf(" Free Fall Detected!\n");
        }
        if (int_status0 & 0x08) {
            printf(" Activity Detected!\n");
        }
        if (int_status2 & 0x02) {
            uint8_t orientation_reg = read_register(&sensor, 0x27);
            uint8_t orientation = orientation_reg & 0x07;
            printf("🔄 Orientation: %s\n", orientation_str[orientation]);
        }

        nanosleep(&sleep_time, NULL);
    }

    close(sensor.i2c_fd);
    printf("\nProgram exited gracefully.\n");
    return 0;
}