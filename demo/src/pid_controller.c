#include "pid_controller.h"
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "PID_CONTROLLER";

void pid_init(PIDController* pid, float kp, float ki, float kd, float max_output) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_output;
    pid->integral_max = max_output * 0.5f;
    pid->enable_integral = true;
    pid->dead_zone = 0.0f;
}

void pid_reset(PIDController* pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

void pid_set_params(PIDController* pid, float kp, float ki, float kd) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(PIDController* pid, float error, float dt) {
    if (!pid) return 0.0f;
    if (dt <= 0.0f) return pid->output;
    
    if (fabsf(error) <= pid->dead_zone && pid->dead_zone > 0.0f) {
        error = 0.0f;
    }
    
    float proportional = pid->kp * error;
    float integral = 0.0f;
    if (pid->enable_integral) {
        if (fabsf(error) < 0.5f) { 
            pid->integral += error * dt;
        }
        if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
        else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
        integral = pid->ki * pid->integral;
    }
    
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;
    
    pid->output = proportional + integral + derivative;
    
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    else if (pid->output < -pid->max_output) pid->output = -pid->max_output;
    
    return pid->output;
}

float pid_update_incremental(PIDController* pid, float error, float dt) {
    return pid_update(pid, error, dt); 
}

void pid_set_integral_limit(PIDController* pid, float integral_max) {
    if (pid) pid->integral_max = integral_max;
}
void pid_set_output_limit(PIDController* pid, float max_output) {
    if (pid) pid->max_output = max_output;
}
void pid_set_dead_zone(PIDController* pid, float dead_zone) {
    if (pid) pid->dead_zone = fabsf(dead_zone);
}
void pid_enable_integral(PIDController* pid, bool enable) {
    if (pid) pid->enable_integral = enable;
}
void pid_get_components(PIDController* pid, float* proportional, float* integral, float* derivative) {}