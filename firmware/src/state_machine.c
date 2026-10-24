#include "state_machine.h"
#include <math.h>

void state_machine_init(SafetyStateMachine *sm) {
    sm->current_state = STATE_BOOT;
    sm->previous_state = STATE_BOOT;
    sm->state_entered_time_ms = 0;
    
    sm->max_tilt_deg = 40.0f;
    sm->max_angular_rate_dps = 250.0f;
    sm->link_timeout_ms = 1200;
    
    sm->is_armed = false;
    sm->emergency_kill_asserted = false;
    sm->last_heartbeat_rx_ms = 0;

    sm->current_state = STATE_STANDBY_DISARMED;
}

void state_machine_update(SafetyStateMachine *sm, float roll_deg, float pitch_deg, 
                          float p_dps, float q_dps, float r_dps, uint32_t now_ms) {
    (void)r_dps;

    if (sm->emergency_kill_asserted) {
        sm->current_state = STATE_EMERGENCY_KILL;
        return;
    }

    // Safety checks
    bool excessive_tilt = (fabsf(roll_deg) > sm->max_tilt_deg) || (fabsf(pitch_deg) > sm->max_tilt_deg);
    bool excessive_rates = (fabsf(p_dps) > sm->max_angular_rate_dps) || (fabsf(q_dps) > sm->max_angular_rate_dps);
    bool link_lost = (now_ms - sm->last_heartbeat_rx_ms > sm->link_timeout_ms) && (sm->last_heartbeat_rx_ms != 0);

    switch (sm->current_state) {
        case STATE_STANDBY_DISARMED:
            if (sm->is_armed) {
                sm->current_state = STATE_ARMED;
                sm->state_entered_time_ms = now_ms;
            }
            break;

        case STATE_ARMED:
            if (!sm->is_armed) {
                sm->current_state = STATE_STANDBY_DISARMED;
            } else if (now_ms - sm->state_entered_time_ms > 200) {
                // Transition to active flight
                sm->current_state = STATE_FLIGHT_NORMAL;
            }
            break;

        case STATE_FLIGHT_NORMAL:
            if (!sm->is_armed) {
                sm->current_state = STATE_STANDBY_DISARMED;
            } else if (excessive_tilt || excessive_rates || link_lost) {
                sm->current_state = STATE_FAILSAFE_ACTIVE;
                sm->state_entered_time_ms = now_ms;
            }
            break;

        case STATE_FAILSAFE_ACTIVE:
            if (!sm->is_armed) {
                sm->current_state = STATE_STANDBY_DISARMED;
            } else if (fabsf(roll_deg) > 75.0f || fabsf(pitch_deg) > 75.0f) {
                // Complete inversion / uncontrolled tumble -> immediate rotor kill
                sm->current_state = STATE_EMERGENCY_KILL;
            }
            break;

        case STATE_EMERGENCY_KILL:
            // Latch in kill state until explicit disarm
            if (!sm->is_armed) {
                sm->emergency_kill_asserted = false;
                sm->current_state = STATE_STANDBY_DISARMED;
            }
            break;

        default:
            sm->current_state = STATE_STANDBY_DISARMED;
            break;
    }
}

void state_machine_request_arm(SafetyStateMachine *sm, bool arm) {
    if (sm->emergency_kill_asserted && arm) {
        return; // Deny arming while kill flag is latched
    }
    sm->is_armed = arm;
}

void state_machine_trigger_emergency_kill(SafetyStateMachine *sm) {
    sm->emergency_kill_asserted = true;
    sm->current_state = STATE_EMERGENCY_KILL;
}

bool state_machine_can_spin_motors(const SafetyStateMachine *sm) {
    return (sm->current_state == STATE_ARMED || 
            sm->current_state == STATE_FLIGHT_NORMAL || 
            sm->current_state == STATE_FAILSAFE_ACTIVE);
}
