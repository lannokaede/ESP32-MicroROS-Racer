#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "motor_controller.h"
#include "imu_driver.h"
#include "pid_controller.h"

#define ROBOT_MAIN_TAG "MAIN"
#define I2C_MASTER_PORT I2C_NUM_0

// 转换系数：TrackAction.wz (电机指令 0.11) -> 物理角速度 (rad/s)
// 估算: 150deg / 4.6s = 32.6deg/s = 0.57rad/s.
// Ratio = 0.57 / 0.11 ≈ 5.2
#define WZ_TO_RAD_S 5.2f 

static PIDController yaw_pid;
static imu_handle_t imu_dev = NULL;
static bool imu_calibrated = false;
static float target_yaw = 0.0f;
static float last_correction_time = 0.0f;

void system_init(void) {
    init_motion_system();
    init_track_controller();
    
    ESP_LOGI(ROBOT_MAIN_TAG, "初始化 IMU...");
    imu_dev = imu_create(I2C_MASTER_PORT, 0x68); // 自动识别 MPU6050/ICM42670
    
    pid_init(&yaw_pid, 3.8f, 0.18f, 0.16f, 0.50f);
    pid_set_dead_zone(&yaw_pid, 0.01f);
}

void calibrate_imu_sensor(void) {
    if (imu_dev && imu_check_connection(imu_dev)) {
        calibrate_imu(imu_dev, 200); // 采样200次
        target_yaw = 0.0f;
        yaw = 0.0f; 
        imu_calibrated = true;
    }
}

void real_time_motion_correction(void) {
    if (!imu_calibrated || !imu_dev) return;

    float current_time = get_current_time();
    float dt = current_time - last_correction_time;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.05f) dt = 0.05f;
    last_correction_time = current_time;

    read_imu_data(imu_dev);
    estimate_attitude(dt);

    bool is_moving = (track_ctl.current_state == STATE_CRUISING || track_ctl.current_state == STATE_ACCELERATING);

    if (is_moving) {
        TrackAction* action = &track_sequence[track_ctl.current_action];
        
        float desired_w = action->wz * WZ_TO_RAD_S;
        
        target_yaw += desired_w * dt;
        target_yaw = normalize_angle(target_yaw);

        float yaw_error = normalize_angle(target_yaw - yaw);

        float correction = pid_update(&yaw_pid, yaw_error, dt);
        
        if (correction > 0.35f) correction = 0.35f;
        if (correction < -0.35f) correction = -0.35f;

        float final_vx = action->vx;
        float final_vy = action->vy;
        float final_wz = action->wz + correction;

        if (final_wz > 1.0f) final_wz = 1.0f;
        if (final_wz < -1.0f) final_wz = -1.0f;

        maintain_velocity(final_vx, final_vy, final_wz);
    } else {
        pid_reset(&yaw_pid);
        target_yaw = yaw; 
    }
}

void display_status(void) {
    static uint32_t last_print = 0;
    if (xTaskGetTickCount() - last_print > pdMS_TO_TICKS(1000)) {
        if (imu_check_connection(imu_dev)) {
            // 打印 Yaw 和 PID 输出，帮你调试
            ESP_LOGI(ROBOT_MAIN_TAG, "[RUN] Act:%d T:%.1f Yaw:%.1f Err:%.2f", 
                track_ctl.current_action,
                get_current_time(),
                yaw * 57.3f, // 显示角度
                (target_yaw - yaw) * 57.3f);
        } else {
            ESP_LOGE(ROBOT_MAIN_TAG, "!!! IMU 断开 !!!");
        }
        last_print = xTaskGetTickCount();
    }
}

void main_control_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (!imu_check_connection(imu_dev)) {
        ESP_LOGE(ROBOT_MAIN_TAG, "等待传感器连接 (SDA=40, SCL=39)...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    calibrate_imu_sensor();
    
    ESP_LOGI(ROBOT_MAIN_TAG, ">>> 3秒后发车 <<<");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 【关键】发车瞬间，再次校准目标方向！
    // 这样无论你把车怎么放，它都会以当前朝向为“正前方”
    if(imu_calibrated) {
        read_imu_data(imu_dev);
        target_yaw = yaw; 
        pid_reset(&yaw_pid);
        last_correction_time = get_current_time();
        ESP_LOGI(ROBOT_MAIN_TAG, "方向已锁定: %.1f", target_yaw * 57.3f);
    }

    start_race();
    
    while (track_ctl.race_started) {
        track_state_machine_update();
        real_time_motion_correction();
        display_status();
        
        if (track_ctl.current_state == STATE_COMPLETED) {
            stop_all_motors();
            vTaskDelay(pdMS_TO_TICKS(200));
            PerformanceMetrics m = generate_performance_report();
            print_performance_report(m);
            track_ctl.race_started = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    system_init();
    xTaskCreate(main_control_task, "main", 8192, NULL, 5, NULL);
}
