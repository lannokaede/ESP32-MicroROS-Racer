#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#define CONTROL_LOOP_INTERVAL 20  

typedef enum {
    STATE_IDLE, STATE_ACCELERATING, STATE_CRUISING, STATE_DECELERATING, 
    STATE_TURNING, STATE_STRAFING, STATE_DIAGONAL, STATE_COMPLETED
} TrackState;

typedef enum {
    ACTION_STRAIGHT, ACTION_TURN_LEFT, ACTION_TURN_RIGHT, ACTION_STOP
} ActionType;

typedef struct {
    ActionType action_type; 
    float vx; float vy; float wz; float duration; 
    const char* description; 
} TrackAction;

typedef struct {
    float total_time;          
    float segment_times[20];   
    float stability_scores[20]; 
    float consistency_score;   
} PerformanceMetrics;

typedef struct {
    TrackState current_state;  
    int current_action;        
    int total_actions;         
    float current_vx; float current_vy; float current_wz;          
    float action_start_time;   
    bool race_started;         
} TrackController;

void init_motion_system(void);
void init_track_controller(void);
void set_robot_velocity(float vx, float vy, float wz);
void stop_all_motors(void);
void track_state_machine_update(void);
float get_current_time(void);
const char* get_state_name(TrackState state);
void maintain_velocity(float target_vx, float target_vy, float target_wz);
void start_race(void);
void calculate_track_total(void);
PerformanceMetrics generate_performance_report(void);
void print_performance_report(PerformanceMetrics metrics);
void init_performance_tracking(void);
void start_segment_timing(int segment_id);
void record_segment_time(int segment_id, float time);

extern TrackController track_ctl;
extern TrackAction track_sequence[];

#endif