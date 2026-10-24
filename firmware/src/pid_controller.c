#include "pid_controller.h"

void pid_init(PidController *pid, float kp, float ki, float kd, 
              float integrator_limit, float output_limit, float d_filter_alpha) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator_limit = integrator_limit;
    pid->output_limit = output_limit;
    pid->d_filter_alpha = d_filter_alpha;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_derivative = 0.0f;
}

float pid_update(PidController *pid, float error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Proportional term
    float p_out = pid->kp * error;

    // Integral term with anti-windup clamping
    pid->integral += error * dt;
    if (pid->integral > pid->integrator_limit) {
        pid->integral = pid->integrator_limit;
    } else if (pid->integral < -pid->integrator_limit) {
        pid->integral = -pid->integrator_limit;
    }
    float i_out = pid->ki * pid->integral;

    // Derivative term with 1st-order low-pass filter
    float raw_derivative = (error - pid->previous_error) / dt;
    float filtered_derivative = pid->d_filter_alpha * raw_derivative + 
                                (1.0f - pid->d_filter_alpha) * pid->previous_derivative;
    float d_out = pid->kd * filtered_derivative;

    pid->previous_error = error;
    pid->previous_derivative = filtered_derivative;

    // Total output with saturation limit
    float total_out = p_out + i_out + d_out;
    if (total_out > pid->output_limit) {
        total_out = pid->output_limit;
    } else if (total_out < -pid->output_limit) {
        total_out = -pid->output_limit;
    }

    return total_out;
}

void pid_reset(PidController *pid) {
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_derivative = 0.0f;
}

void pid_set_gains(PidController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
