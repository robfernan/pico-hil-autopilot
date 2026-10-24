#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Tunable gains
    float kp;
    float ki;
    float kd;
    
    // Limits
    float integrator_limit;
    float output_limit;
    
    // Filter coefficient for derivative (alpha in [0, 1])
    float d_filter_alpha;
    
    // State memory
    float integral;
    float previous_error;
    float previous_derivative;
} PidController;

void pid_init(PidController *pid, float kp, float ki, float kd, 
              float integrator_limit, float output_limit, float d_filter_alpha);

float pid_update(PidController *pid, float error, float dt);
void pid_reset(PidController *pid);
void pid_set_gains(PidController *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
