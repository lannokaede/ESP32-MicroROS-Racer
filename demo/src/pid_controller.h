#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    float kp;           
    float ki;           
    float kd;           
    float integral;     
    float prev_error;   
    float output;       
    float max_output;   
    float integral_max; 
    bool enable_integral; 
    float dead_zone;    
} PIDController;

void pid_init(PIDController* pid, float kp, float ki, float kd, float max_output);
void pid_reset(PIDController* pid);
void pid_set_params(PIDController* pid, float kp, float ki, float kd);
float pid_update(PIDController* pid, float error, float dt);
float pid_update_incremental(PIDController* pid, float error, float dt);
void pid_set_integral_limit(PIDController* pid, float integral_max);
void pid_set_output_limit(PIDController* pid, float max_output);
void pid_set_dead_zone(PIDController* pid, float dead_zone);
void pid_enable_integral(PIDController* pid, bool enable);
void pid_get_components(PIDController* pid, float* proportional, float* integral, float* derivative);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H