#ifndef COCKPIT_H
#define COCKPIT_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>
#include "protocol.h"

#define WINDOW_WIDTH  1024
#define WINDOW_HEIGHT 620

#define HISTORY_LEN 200

typedef struct {
    // Current live telemetry
    TelemetryFastPacket fast;
    TelemetrySlowPacket slow;
    
    // Derived UI state
    float target_pitch;
    float target_roll;
    float target_yaw;
    
    // Waveform history for strip chart
    float pitch_history[HISTORY_LEN];
    float roll_history[HISTORY_LEN];
    float target_pitch_history[HISTORY_LEN];
    int history_index;
    
    // Connection & HIL state
    bool is_hardware_connected;
    char port_name[64];
    uint32_t last_packet_time;
    uint32_t packets_received;
    uint32_t crc_errors;
    
    // Fault injection status
    HilFaultType injected_fault;
    uint32_t fault_start_time;
    
    // Simulated physics state (when hardware is running or in simulated mode)
    float sim_p, sim_q, sim_r; // angular velocities
    float sim_thrust;
} CockpitState;

void cockpit_init(CockpitState *state);
void cockpit_update(CockpitState *state, float dt);
void cockpit_render(SDL_Renderer *renderer, CockpitState *state);
void cockpit_inject_fault(CockpitState *state, HilFaultType fault);
void cockpit_toggle_arm(CockpitState *state);

#endif // COCKPIT_H
