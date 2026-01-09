#include "motor_controller.h"
#include "imu_driver.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MOTOR_FR 0
#define MOTOR_FL 1
#define MOTOR_RR 2
#define MOTOR_RL 3

static const int MOTOR_PINS[4][2] = {
    {4, 5},   {15, 16}, {9, 10},  {13, 14}
};

static const ledc_channel_config_t motor_channels[8] = {
    {.channel = LEDC_CHANNEL_0, .gpio_num = MOTOR_PINS[0][0], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_1, .gpio_num = MOTOR_PINS[0][1], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_2, .gpio_num = MOTOR_PINS[1][0], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_3, .gpio_num = MOTOR_PINS[1][1], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_4, .gpio_num = MOTOR_PINS[2][0], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_5, .gpio_num = MOTOR_PINS[2][1], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_6, .gpio_num = MOTOR_PINS[3][0], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
    {.channel = LEDC_CHANNEL_7, .gpio_num = MOTOR_PINS[3][1], .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0}
};

TrackController track_ctl;
TickType_t system_start_time;
static PerformanceMetrics performance_data;
static bool segment_completed[20] = {false};
static float current_segment_jitter_sum = 0.0f;
static int current_segment_samples = 0;

TrackAction track_sequence[] = {
    {ACTION_STRAIGHT, 0.8, 0.0, 0.0, 3.125, "直道2.5m"},
    {ACTION_TURN_RIGHT, 0.8, 0.0, -0.236, 2.125, "右转150"},
    {ACTION_TURN_RIGHT, 0.8, 0.0, -0.236, 1.275, "右转90"},
    {ACTION_TURN_LEFT, 0.8, 0.0, 0.236, 0.850, "左转60"},
    {ACTION_STRAIGHT, 0.8, 0.0, 0.0, 0.625, "直道0.5m"},
    {ACTION_TURN_LEFT, 0.8, 0.0, 0.184, 1.170, "左转64"},
    {ACTION_TURN_RIGHT, 0.8, 0.0, -0.236, 2.185, "右转154"},
    {ACTION_STRAIGHT, 0.8, 0.0, 0.0, 0.250, "直道0.2m"},
    {ACTION_TURN_RIGHT, 0.8, 0.0, -0.236, 1.275, "右转90"},
    {ACTION_STRAIGHT, 0.8, 0.0, 0.0, 0.625, "直道0.5m"},

    {ACTION_STOP, 0.0, 0.0, 0.0, 1.0, "停止"}
};

void motor_driver_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = 1000, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    for (int i = 0; i < 8; i++) ledc_channel_config(&motor_channels[i]);
}

void set_motor_speed(int motor_id, float speed) {
    if (motor_id < 0 || motor_id > 3) return;
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    if (motor_id == MOTOR_FL || motor_id == MOTOR_RL) speed = -speed;
    
    uint32_t duty = (uint32_t)(fabsf(speed) * 255);
    if (speed >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channels[motor_id*2].channel, duty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channels[motor_id*2+1].channel, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channels[motor_id*2].channel, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channels[motor_id*2+1].channel, duty);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_channels[motor_id*2].channel);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_channels[motor_id*2+1].channel);
}

void set_robot_velocity(float vx, float vy, float wz) {
    float wheel_speeds[4];
    float k = 0.75f;
    wheel_speeds[0] = vx - vy - k * wz; 
    wheel_speeds[1] = vx + vy + k * wz; 
    wheel_speeds[2] = vx + vy - k * wz; 
    wheel_speeds[3] = vx - vy + k * wz; 
    
    set_motor_speed(MOTOR_FL, wheel_speeds[0]);
    set_motor_speed(MOTOR_FR, wheel_speeds[1]);
    set_motor_speed(MOTOR_RL, wheel_speeds[2]);
    set_motor_speed(MOTOR_RR, wheel_speeds[3]);
    
    track_ctl.current_vx = vx;
    track_ctl.current_vy = vy;
    track_ctl.current_wz = wz;
}

void stop_all_motors() { set_robot_velocity(0,0,0); }
void init_motion_system() { motor_driver_init(); stop_all_motors(); }
float get_current_time() { return (float)(xTaskGetTickCount() - system_start_time) * portTICK_PERIOD_MS / 1000.0; }

void init_track_controller() {
    memset(&track_ctl, 0, sizeof(TrackController));
    track_ctl.current_state = STATE_IDLE;
    track_ctl.total_actions = sizeof(track_sequence) / sizeof(TrackAction);
    system_start_time = xTaskGetTickCount();
    memset(&performance_data, 0, sizeof(PerformanceMetrics));
    memset(segment_completed, 0, sizeof(segment_completed));
    current_segment_jitter_sum = 0;
    current_segment_samples = 0;
}

void start_race() { 
    if(!track_ctl.race_started) {
        track_ctl.race_started = true; 
        current_segment_jitter_sum = 0;
        current_segment_samples = 0;
    }
}
void calculate_track_total() { }

void record_segment_data(int id, float time) {
    if (id >= 20 || segment_completed[id]) return;
    performance_data.segment_times[id] = time;
    segment_completed[id] = true;
    
    float avg_jitter = 0.0f;
    if (current_segment_samples > 0) {
        avg_jitter = current_segment_jitter_sum / current_segment_samples;
    }
    
    float score;
    if (avg_jitter < 0.0001f) { score = 0.00f; } 
    else {
        score = 1.0f - (avg_jitter * 1.5f);
        if (score < 0.0f) score = 0.0f;
        if (score > 1.0f) score = 1.0f;
    }
    performance_data.stability_scores[id] = score;
    current_segment_jitter_sum = 0;
    current_segment_samples = 0;
}

void track_state_machine_update() {
    float current_time = get_current_time();
    float elapsed_time = current_time - track_ctl.action_start_time;
    
    if (track_ctl.current_state == STATE_CRUISING || track_ctl.current_state == STATE_ACCELERATING) {
        current_segment_jitter_sum += get_gyro_instability();
        current_segment_samples++;
    }

    if (track_ctl.current_action >= track_ctl.total_actions) {
        track_ctl.current_state = STATE_COMPLETED;
    }
    
    switch (track_ctl.current_state) {
        case STATE_IDLE:
            if (track_ctl.race_started) {
                track_ctl.current_state = STATE_ACCELERATING;
                track_ctl.current_action = 0;
                track_ctl.action_start_time = current_time;
            }
            break;
        case STATE_ACCELERATING:
        case STATE_CRUISING:
            {
                float vx = track_sequence[track_ctl.current_action].vx;
                float vy = track_sequence[track_ctl.current_action].vy;
                float wz = track_sequence[track_ctl.current_action].wz;
                set_robot_velocity(vx, vy, wz);
                track_ctl.current_state = STATE_CRUISING;
                if (elapsed_time >= track_sequence[track_ctl.current_action].duration) {
                    record_segment_data(track_ctl.current_action, elapsed_time);
                    track_ctl.current_action++;
                    track_ctl.action_start_time = current_time;
                    if (track_ctl.current_action >= track_ctl.total_actions) track_ctl.current_state = STATE_COMPLETED;
                }
            }
            break;
        case STATE_COMPLETED:
            stop_all_motors();
            break;
        default:
            track_ctl.current_state = STATE_CRUISING;
            break;
    }
}

void maintain_velocity(float vx, float vy, float wz) { set_robot_velocity(vx, vy, wz); }
const char* get_state_name(TrackState state) { return "RUN"; } 

PerformanceMetrics generate_performance_report() {
    performance_data.total_time = get_current_time();
    return performance_data;
}

void print_performance_report(PerformanceMetrics metrics) {
    printf("\n");
    ESP_LOGI("PERF", "==================== 真实性能报告 ====================");
    ESP_LOGI("PERF", "总耗时: %.2f秒", metrics.total_time);
    ESP_LOGI("PERF", "--------------------------------------------------");
    ESP_LOGI("PERF", "段号   耗时(s)   稳定性(0.00=异常)");
    ESP_LOGI("PERF", "--------------------------------------------------");
    for (int i = 0; i < track_ctl.total_actions; i++) {
        if (segment_completed[i]) {
            ESP_LOGI("PERF", " %2d     %6.2f      %5.2f", i, metrics.segment_times[i], metrics.stability_scores[i]);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    ESP_LOGI("PERF", "==================================================");
}

void accelerate_to_target(float target_vx, float target_vy, float target_wz) { set_robot_velocity(target_vx, target_vy, target_wz); }
void decelerate_to_stop() { stop_all_motors(); }
void init_performance_tracking(void) {
    memset(&performance_data, 0, sizeof(PerformanceMetrics));
    memset(segment_completed, 0, sizeof(segment_completed));
    current_segment_jitter_sum = 0;
    current_segment_samples = 0;
}
void start_segment_timing(int segment_id) {}
void record_segment_time(int segment_id, float time) {}
