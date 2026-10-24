#ifndef HIL_ENGINE_H
#define HIL_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"
#include "state_machine.h"
#include "pid_controller.h"
#include "physics_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PhysicsSimulator6DOF sim;
    SafetyStateMachine sm;
    
    // Cascaded PID controllers
    PidController pid_roll_angle;
    PidController pid_roll_rate;
    PidController pid_pitch_angle;
    PidController pid_pitch_rate;
    PidController pid_yaw_rate;
    
    // Setpoints
    float target_roll;
    float target_pitch;
    float target_yaw;
    float target_altitude;
    
    // Current motor outputs (0 - 1000)
    uint16_t motor_pwm[4];
    
    // Telemetry cache shared between Core 0 and Core 1
    TelemetryFastPacket fast_packet;
    TelemetrySlowPacket slow_packet;
    
    // Loop metrics
    uint32_t loop_counter;
    uint32_t last_time_us;
} HilEngine;

void hil_engine_init(HilEngine *engine);
void hil_engine_core0_step(HilEngine *engine, float dt, uint32_t now_ms);
void hil_engine_core1_process_telemetry(HilEngine *engine, uint32_t now_ms);
void hil_engine_handle_command(HilEngine *engine, uint8_t msg_id, const uint8_t *payload, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // HIL_ENGINE_H
