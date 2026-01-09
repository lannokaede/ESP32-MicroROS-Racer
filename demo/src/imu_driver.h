#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void* imu_handle_t;
#define IMU_I2C_ADDRESS 0x68 // I2C地址通常兼容

typedef struct { float x; float y; float z; } imu_accel_t;
typedef struct { float x; float y; float z; } imu_gyro_t;

extern float yaw; // 全局 Yaw 角

// 初始化
imu_handle_t imu_create(int i2c_port, uint8_t i2c_address);
bool imu_check_connection(imu_handle_t imu);

// 数据处理
void read_imu_data(imu_handle_t imu);
void estimate_attitude(float dt);
void calibrate_imu(imu_handle_t imu, int sample_count);

// 获取真实抖动值
float get_gyro_instability(void);
float imu_get_gz_filtered(void);
float normalize_angle(float angle);

#ifdef __cplusplus
}
#endif
#endif
