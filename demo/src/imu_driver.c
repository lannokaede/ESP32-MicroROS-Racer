#include "imu_driver.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>

// ESP32-S3 引脚
#define I2C_MASTER_SCL_IO 39
#define I2C_MASTER_SDA_IO 40
#define I2C_MASTER_FREQ_HZ 100000 
#define I2C_MASTER_PORT I2C_NUM_0

// === MPU6050 寄存器 ===
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_GYRO_XOUT  0x43

// === ICM-42670-P 寄存器 (ID=0x67) ===
#define ICM_REG_PWR_MGMT0  0x1F
#define ICM_REG_GYRO_CONFIG0 0x20
#define ICM_REG_GYRO_DATA_X1 0x11 
#define ICM_REG_WHO_AM_I   0x75

const float IMU_GYRO_SCALE = 1.0f / 131.0f; // 250dps

// 全局变量
float yaw = 0.0f;
float gz_filtered = 0.0f;
float gyro_offset_z = 0.0f;
static float current_instability_metric = 0.0f; 

// 芯片类型标记: 0=Unknown, 1=MPU6050, 2=ICM-42670
static int detected_chip_type = 0; 

typedef struct { int i2c_port; uint8_t i2c_address; } imu_device_t;

static esp_err_t i2c_read_bytes(imu_device_t* dev, uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_address << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write_byte(imu_device_t* dev, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// 初始化 ICM-42670-P
static bool init_icm42670(imu_device_t* dev) {
    ESP_LOGI("IMU", "检测到 ICM-42670-P (ID:0x67)，正在初始化...");
    
    // 1. 设置 Gyro 和 Accel 为 Low Noise Mode (开启)
    // Register 0x1F (PWR_MGMT0): bits 3:2=GyroMode(11=LN), 1:0=AccelMode(11=LN) -> 0x0F
    i2c_write_byte(dev, ICM_REG_PWR_MGMT0, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. 配置 Gyro 量程为 +/- 250 dps (与代码兼容)
    // Register 0x20 (GYRO_CONFIG0): bits 6:5=FS_SEL(11=250dps), 3:0=ODR -> 0x60
    i2c_write_byte(dev, ICM_REG_GYRO_CONFIG0, 0x60);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI("IMU", "ICM-42670 初始化完成");
    return true;
}

// 初始化 MPU6050
static bool init_mpu6050(imu_device_t* dev) {
    ESP_LOGI("IMU", "检测到 MPU6050 (ID:0x68)，正在初始化...");
    // 复位
    i2c_write_byte(dev, MPU_REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    // 唤醒
    i2c_write_byte(dev, MPU_REG_PWR_MGMT_1, 0x00);
    return true;
}

imu_handle_t imu_create(int i2c_port, uint8_t i2c_address) {
    static bool init_done = false;
    if (!init_done) {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
        i2c_param_config(I2C_MASTER_PORT, &conf);
        i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
        init_done = true;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    imu_device_t* dev = (imu_device_t*)malloc(sizeof(imu_device_t));
    dev->i2c_port = i2c_port;
    dev->i2c_address = i2c_address;

    // 读取 WHO_AM_I (Register 0x75 对于两者都是同一个地址)
    uint8_t who_am_i = 0;
    if (i2c_read_bytes(dev, ICM_REG_WHO_AM_I, &who_am_i, 1) != ESP_OK) {
        // 尝试备用地址
        dev->i2c_address = 0x69;
        i2c_read_bytes(dev, ICM_REG_WHO_AM_I, &who_am_i, 1);
    }

    ESP_LOGI("IMU", "I2C 连接成功, ID: 0x%02X", who_am_i);

    if (who_am_i == 0x67) {
        detected_chip_type = 2; // ICM-42670
        init_icm42670(dev);
    } else if (who_am_i == 0x68) {
        detected_chip_type = 1; // MPU6050
        init_mpu6050(dev);
    } else {
        ESP_LOGE("IMU", "未知芯片 ID! 可能无法读取数据");
        detected_chip_type = 0;
    }

    return (imu_handle_t)dev;
}

bool imu_check_connection(imu_handle_t imu) {
    if (!imu) return false;
    uint8_t dummy;
    return i2c_read_bytes((imu_device_t*)imu, ICM_REG_WHO_AM_I, &dummy, 1) == ESP_OK;
}

void read_imu_data(imu_handle_t imu) {
    if (!imu) return;
    
    uint8_t buf[6];
    uint8_t start_reg;

    if (detected_chip_type == 2) {
        // ICM-42670-P: Gyro 数据从 0x11 开始 (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
        start_reg = ICM_REG_GYRO_DATA_X1;
    } else {
        // MPU6050: Gyro 数据从 0x43 开始
        start_reg = MPU_REG_GYRO_XOUT;
    }

    if (i2c_read_bytes((imu_device_t*)imu, start_reg, buf, 6) != ESP_OK) {
        return; 
    }

    // 两者都是 Big Endian (高位在前)
    // Z 轴数据在 buf[4] 和 buf[5]
    int16_t raw_gz = (int16_t)((buf[4] << 8) | buf[5]);
    
    // 转换为 deg/s (两者都配置为 +/- 250dps -> 131 LSB/dps)
    float gz_deg = raw_gz * IMU_GYRO_SCALE;
    
    // 转弧度/秒
    float gz_rad = (gz_deg - gyro_offset_z) * (M_PI / 180.0f);
    
    // 滤波
    gz_filtered = 0.7f * gz_filtered + 0.3f * gz_rad;
    
    // 更新抖动值
    current_instability_metric = fabsf(gz_rad);
}

void estimate_attitude(float dt) {
    yaw += gz_filtered * dt;
    yaw = normalize_angle(yaw);
}

float get_gyro_instability(void) {
    return current_instability_metric;
}

float imu_get_gz_filtered(void) {
    return gz_filtered;
}

void calibrate_imu(imu_handle_t imu, int sample_count) {
    if (!imu) return;
    ESP_LOGI("IMU", "开始校准...");
    float sum_z = 0;
    int count = 0;
    
    // 读取几次丢弃，稳定数据
    for(int k=0; k<10; k++) read_imu_data(imu);

    for(int i=0; i<sample_count; i++) {
        uint8_t buf[6];
        uint8_t reg = (detected_chip_type == 2) ? ICM_REG_GYRO_DATA_X1 : MPU_REG_GYRO_XOUT;
        
        if (i2c_read_bytes((imu_device_t*)imu, reg, buf, 6) == ESP_OK) {
            int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);
            sum_z += (raw_z * IMU_GYRO_SCALE);
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    if (count > 0) {
        gyro_offset_z = sum_z / count;
        ESP_LOGI("IMU", "校准完成. Z轴零偏: %.2f deg/s", gyro_offset_z);
    }
}

float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
