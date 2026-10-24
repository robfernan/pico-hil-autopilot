#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    VehicleState current_state;
    VehicleState previous_state;
    uint32_t state_entered_time_ms;
    
    // Safety thresholds
    float max_tilt_deg;          // Maximum allowable bank angle before failsafe (e.g., 45 deg)
    float max_angular_rate_dps;  // Maximum allowable angular rate before failsafe (e.g., 300 dps)
    uint32_t link_timeout_ms;    // Watchdog timeout for GCS link loss (e.g., 1000 ms)
    
    // Internal counters & flags
    bool is_armed;
    bool emergency_kill_asserted;
    uint32_t last_heartbeat_rx_ms;
} SafetyStateMachine;

void state_machine_init(SafetyStateMachine *sm);
void state_machine_update(SafetyStateMachine *sm, float roll_deg, float pitch_deg, 
                          float p_dps, float q_dps, float r_dps, uint32_t now_ms);

void state_machine_request_arm(SafetyStateMachine *sm, bool arm);
void state_machine_trigger_emergency_kill(SafetyStateMachine *sm);
bool state_machine_can_spin_motors(const SafetyStateMachine *sm);

#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H
