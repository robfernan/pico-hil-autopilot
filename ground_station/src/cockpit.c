#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "cockpit.h"

#define DEG_TO_RAD (3.1415926535f / 180.0f)

#include "font8x8_basic.h"

static void draw_char(SDL_Renderer *renderer, char c, int x, int y, int scale, SDL_Color color) {
    uint8_t uc = (uint8_t)c & 0x7F;
    const char *glyph = font8x8_basic[uc];
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            if ((glyph[row] >> col) & 1) {
                SDL_Rect pixel = { x + col * scale, y + row * scale, scale, scale };
                SDL_RenderFillRect(renderer, &pixel);
            }
        }
    }
}

static void draw_string(SDL_Renderer *renderer, const char *str, int x, int y, int scale, SDL_Color color) {
    int cur_x = x;
    while (*str) {
        draw_char(renderer, *str, cur_x, y, scale, color);
        cur_x += 8 * scale + scale; // add 1px spacing
        str++;
    }
}

static void draw_filled_circle(SDL_Renderer *renderer, int cx, int cy, int radius, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx*dx + dy*dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }
}

static void draw_line_thick(SDL_Renderer *renderer, int x1, int y1, int x2, int y2, int thickness) {
    for (int i = -thickness / 2; i <= thickness / 2; ++i) {
        SDL_RenderDrawLine(renderer, x1 + i, y1, x2 + i, y2);
        SDL_RenderDrawLine(renderer, x1, y1 + i, x2, y2 + i);
    }
}

void cockpit_init(CockpitState *state) {
    memset(state, 0, sizeof(CockpitState));
    state->fast.state = STATE_STANDBY_DISARMED;
    state->fast.motor_pwm[0] = 0;
    state->fast.motor_pwm[1] = 0;
    state->fast.motor_pwm[2] = 0;
    state->fast.motor_pwm[3] = 0;
    state->fast.altitude_cm = 120; // 1.2 meters
    state->slow.core0_cpu_pct = 42;
    state->slow.core1_cpu_pct = 18;
    state->slow.free_heap_bytes = 184320;
    state->slow.loop_jitter_us = 4;
}

void cockpit_update(CockpitState *state, float dt) {
    // If not connected to hardware, run dynamic physics simulation
    if (!state->is_hardware_connected) {
        if (state->fast.state == STATE_ARMED || state->fast.state == STATE_FLIGHT_NORMAL) {
            state->sim_thrust += (650.0f - state->sim_thrust) * dt * 2.0f;
            state->fast.altitude_cm += (int32_t)((state->sim_thrust - 500.0f) * dt * 1.5f);
            if (state->fast.altitude_cm < 0) state->fast.altitude_cm = 0;
        } else {
            state->sim_thrust += (0.0f - state->sim_thrust) * dt * 3.0f;
            state->fast.altitude_cm += (0 - state->fast.altitude_cm) * dt * 4.0f;
        }

        // Simulate attitude dynamics with gentle flight oscillations + pilot steering
        float t = SDL_GetTicks() / 1000.0f;
        float base_pitch = sinf(t * 1.2f) * 4.0f + state->target_pitch;
        float base_roll  = cosf(t * 0.9f) * 5.0f + state->target_roll;
        
        if (state->injected_fault == FAULT_GYRO_BIAS) {
            base_roll += 24.0f; // drift to right
            state->fast.state = STATE_FAILSAFE_ACTIVE;
        } else if (state->injected_fault == FAULT_MOTOR_3_CUT) {
            base_pitch -= 16.0f;
            base_roll  -= 20.0f;
            state->fast.state = STATE_FAILSAFE_ACTIVE;
        }

        state->fast.pitch = (int16_t)(base_pitch * 100.0f);
        state->fast.roll  = (int16_t)(base_roll * 100.0f);
        state->fast.yaw   = (int16_t)(fmodf(t * 15.0f, 360.0f) * 100.0f);

        // Update motor PWMs
        uint16_t m_base = (uint16_t)state->sim_thrust;
        state->fast.motor_pwm[0] = m_base + (uint16_t)(base_pitch * 10.0f) - (uint16_t)(base_roll * 10.0f);
        state->fast.motor_pwm[1] = m_base + (uint16_t)(base_pitch * 10.0f) + (uint16_t)(base_roll * 10.0f);
        state->fast.motor_pwm[2] = (state->injected_fault == FAULT_MOTOR_3_CUT) ? 0 : 
                                   (m_base - (uint16_t)(base_pitch * 10.0f) - (uint16_t)(base_roll * 10.0f));
        state->fast.motor_pwm[3] = m_base - (uint16_t)(base_pitch * 10.0f) + (uint16_t)(base_roll * 10.0f);
        
        for (int i = 0; i < 4; ++i) {
            if (state->fast.motor_pwm[i] > 1000) state->fast.motor_pwm[i] = 1000;
        }
    }

    // Record waveform history
    state->pitch_history[state->history_index] = (float)state->fast.pitch / 100.0f;
    state->roll_history[state->history_index]  = (float)state->fast.roll / 100.0f;
    state->target_pitch_history[state->history_index] = state->target_pitch;
    state->history_index = (state->history_index + 1) % HISTORY_LEN;
}

static void render_artificial_horizon(SDL_Renderer *renderer, CockpitState *state, int cx, int cy, int radius) {
    float roll_deg  = (float)state->fast.roll / 100.0f;
    float pitch_deg = (float)state->fast.pitch / 100.0f;
    if (state->is_hardware_connected) {
        roll_deg  += state->target_roll;
        pitch_deg += state->target_pitch;
    }
    float roll_rad  = roll_deg * DEG_TO_RAD;

    // Outer Bezel
    draw_filled_circle(renderer, cx, cy, radius + 10, (SDL_Color){32, 40, 52, 255});
    draw_filled_circle(renderer, cx, cy, radius + 2, (SDL_Color){14, 20, 26, 255});

    // Circular horizon viewport
    for (int y = -radius; y <= radius; y++) {
        int half_w = (int)sqrtf((float)(radius * radius - y * y));
        for (int x = -half_w; x <= half_w; x++) {
            float rot_y = (float)y * cosf(roll_rad) - (float)x * sinf(roll_rad);
            float pitch_offset = pitch_deg * 2.2f;

            if (rot_y + pitch_offset > 0) {
                SDL_SetRenderDrawColor(renderer, 64, 40, 32, 255); // Earth
            } else {
                SDL_SetRenderDrawColor(renderer, 20, 50, 92, 255); // Sky
            }
            SDL_RenderDrawPoint(renderer, cx + x, cy + y);
        }
    }

    // Horizon line
    float hx1 = -radius * 0.85f;
    float hx2 =  radius * 0.85f;
    float hy  = -pitch_deg * 2.2f;

    int px1 = cx + (int)(hx1 * cosf(roll_rad) - hy * sinf(roll_rad));
    int py1 = cy + (int)(hx1 * sinf(roll_rad) + hy * cosf(roll_rad));
    int px2 = cx + (int)(hx2 * cosf(roll_rad) - hy * sinf(roll_rad));
    int py2 = cy + (int)(hx2 * sinf(roll_rad) + hy * cosf(roll_rad));

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    draw_line_thick(renderer, px1, py1, px2, py2, 3);

    // Pitch Ladder (-20, -10, +10, +20 deg)
    for (int deg = -20; deg <= 20; deg += 10) {
        if (deg == 0) continue;
        float line_y = -(pitch_deg - (float)deg) * 2.2f;
        float bar_w = (deg % 20 == 0) ? 36.0f : 20.0f;

        int lx1 = cx + (int)(-bar_w * cosf(roll_rad) - line_y * sinf(roll_rad));
        int ly1 = cy + (int)(-bar_w * sinf(roll_rad) + line_y * cosf(roll_rad));
        int lx2 = cx + (int)( bar_w * cosf(roll_rad) - line_y * sinf(roll_rad));
        int ly2 = cy + (int)( bar_w * sinf(roll_rad) + line_y * cosf(roll_rad));

        int dist_sq = (lx1 - cx)*(lx1 - cx) + (ly1 - cy)*(ly1 - cy);
        if (dist_sq < (radius - 15) * (radius - 15)) {
            SDL_SetRenderDrawColor(renderer, 220, 220, 220, 220);
            draw_line_thick(renderer, lx1, ly1, lx2, ly2, 2);
        }
    }

    // Fixed Aircraft Reference Symbol (Bright Amber/Yellow)
    SDL_SetRenderDrawColor(renderer, 255, 196, 0, 255);
    draw_line_thick(renderer, cx - 45, cy, cx - 15, cy, 4);
    draw_line_thick(renderer, cx - 15, cy, cx - 15, cy + 12, 4);
    draw_line_thick(renderer, cx + 15, cy, cx + 45, cy, 4);
    draw_line_thick(renderer, cx + 15, cy, cx + 15, cy + 12, 4);
    draw_filled_circle(renderer, cx, cy, 4, (SDL_Color){255, 196, 0, 255});

    // Roll indicator pointer at top
    int rx = cx + (int)((radius - 6) * sinf(roll_rad));
    int ry = cy - (int)((radius - 6) * cosf(roll_rad));
    draw_filled_circle(renderer, rx, ry, 5, (SDL_Color){255, 60, 60, 255});

    // PFD Digital Readouts
    char roll_str[32], pitch_str[32], alt_str[32], hdg_str[32];
    snprintf(roll_str, sizeof(roll_str), "ROLL: %+.1f DEG", roll_deg);
    snprintf(pitch_str, sizeof(pitch_str), "PITCH: %+.1f DEG", pitch_deg);
    snprintf(alt_str, sizeof(alt_str), "ALT: %.2f M", (float)state->fast.altitude_cm / 100.0f);
    snprintf(hdg_str, sizeof(hdg_str), "HDG: %03d DEG", (int)(state->fast.yaw / 100));

    draw_string(renderer, roll_str, cx - 75, cy + radius + 18, 1, (SDL_Color){0, 229, 255, 255});
    draw_string(renderer, pitch_str, cx - 80, cy + radius + 34, 1, (SDL_Color){0, 229, 255, 255});
    draw_string(renderer, alt_str, cx - 55, cy - radius - 26, 1, (SDL_Color){0, 230, 118, 255});
    draw_string(renderer, hdg_str, cx - 60, cy - radius - 42, 1, (SDL_Color){255, 214, 0, 255});
}

static void render_strip_chart(SDL_Renderer *renderer, CockpitState *state, int x, int y, int w, int h) {
    SDL_Rect bg = {x, y, w, h};
    SDL_SetRenderDrawColor(renderer, 18, 24, 32, 255);
    SDL_RenderFillRect(renderer, &bg);
    SDL_SetRenderDrawColor(renderer, 38, 50, 66, 255);
    SDL_RenderDrawRect(renderer, &bg);

    // Title & Legend
    draw_string(renderer, "ATTITUDE DYNAMICS", x + 15, y + 12, 1, (SDL_Color){200, 210, 225, 255});
    draw_string(renderer, "- PITCH (CYAN)", x + 240, y + 12, 1, (SDL_Color){0, 229, 255, 255});
    draw_string(renderer, "- ROLL (PINK)", x + 400, y + 12, 1, (SDL_Color){255, 64, 129, 255});

    // Center Zero Line
    int mid_y = y + h / 2 + 10;
    SDL_SetRenderDrawColor(renderer, 60, 75, 95, 255);
    SDL_RenderDrawLine(renderer, x, mid_y, x + w, mid_y);

    // Grid lines (+-10, +-20 deg)
    SDL_SetRenderDrawColor(renderer, 30, 42, 56, 180);
    SDL_RenderDrawLine(renderer, x, mid_y - 45, x + w, mid_y - 45);
    SDL_RenderDrawLine(renderer, x, mid_y + 45, x + w, mid_y + 45);

    draw_string(renderer, "+15 DEG", x + 8, mid_y - 50, 1, (SDL_Color){90, 110, 130, 255});
    draw_string(renderer, "  0 DEG", x + 8, mid_y - 4, 1, (SDL_Color){120, 140, 160, 255});
    draw_string(renderer, "-15 DEG", x + 8, mid_y + 42, 1, (SDL_Color){90, 110, 130, 255});

    // Draw Pitch History (Neon Cyan)
    SDL_SetRenderDrawColor(renderer, 0, 229, 255, 255);
    for (int i = 0; i < HISTORY_LEN - 1; ++i) {
        int idx1 = (state->history_index + i) % HISTORY_LEN;
        int idx2 = (state->history_index + i + 1) % HISTORY_LEN;

        int px1 = x + (i * w) / HISTORY_LEN;
        int py1 = mid_y - (int)(state->pitch_history[idx1] * 3.2f);
        int px2 = x + ((i + 1) * w) / HISTORY_LEN;
        int py2 = mid_y - (int)(state->pitch_history[idx2] * 3.2f);

        if (py1 >= y && py1 < y + h && py2 >= y && py2 < y + h) {
            SDL_RenderDrawLine(renderer, px1, py1, px2, py2);
        }
    }

    // Draw Roll History (Neon Magenta)
    SDL_SetRenderDrawColor(renderer, 255, 64, 129, 255);
    for (int i = 0; i < HISTORY_LEN - 1; ++i) {
        int idx1 = (state->history_index + i) % HISTORY_LEN;
        int idx2 = (state->history_index + i + 1) % HISTORY_LEN;

        int px1 = x + (i * w) / HISTORY_LEN;
        int py1 = mid_y - (int)(state->roll_history[idx1] * 3.2f);
        int px2 = x + ((i + 1) * w) / HISTORY_LEN;
        int py2 = mid_y - (int)(state->roll_history[idx2] * 3.2f);

        if (py1 >= y && py1 < y + h && py2 >= y && py2 < y + h) {
            SDL_RenderDrawLine(renderer, px1, py1, px2, py2);
        }
    }
}

static void render_motor_gauges(SDL_Renderer *renderer, CockpitState *state, int x, int y, int w, int h) {
    SDL_Rect bg = {x, y, w, h};
    SDL_SetRenderDrawColor(renderer, 18, 24, 32, 255);
    SDL_RenderFillRect(renderer, &bg);
    SDL_SetRenderDrawColor(renderer, 38, 50, 66, 255);
    SDL_RenderDrawRect(renderer, &bg);

    draw_string(renderer, "QUADROTOR PROPULSION - PWM PERMILLE", x + 15, y + 12, 1, (SDL_Color){200, 210, 225, 255});

    int bar_w = (w - 70) / 4;
    int max_bar_h = h - 60;
    const char *labels[4] = {"M1:FL", "M2:FR", "M3:RL", "M4:RR"};

    for (int i = 0; i < 4; ++i) {
        int bx = x + 20 + i * (bar_w + 12);
        int by = y + 35;

        // Background slot
        SDL_Rect slot = {bx, by, bar_w, max_bar_h};
        SDL_SetRenderDrawColor(renderer, 26, 34, 46, 255);
        SDL_RenderFillRect(renderer, &slot);

        // Fill level
        float pct = (float)state->fast.motor_pwm[i] / 1000.0f;
        int fill_h = (int)(pct * max_bar_h);
        SDL_Rect fill = {bx, by + max_bar_h - fill_h, bar_w, fill_h};

        if (i == 2 && state->injected_fault == FAULT_MOTOR_3_CUT) {
            SDL_SetRenderDrawColor(renderer, 255, 40, 40, 255); // Dead motor
        } else if (pct > 0.85f) {
            SDL_SetRenderDrawColor(renderer, 255, 171, 0, 255); // Warning amber
        } else {
            SDL_SetRenderDrawColor(renderer, 0, 230, 118, 255); // Nominal green
        }
        SDL_RenderFillRect(renderer, &fill);

        // Label
        draw_string(renderer, labels[i], bx + 12, by + max_bar_h + 8, 1, (SDL_Color){170, 190, 210, 255});

        // Numeric percentage
        char pct_str[16];
        snprintf(pct_str, sizeof(pct_str), "%d%%", (int)(pct * 100));
        draw_string(renderer, pct_str, bx + 18, by + max_bar_h - fill_h - 14, 1, (SDL_Color){255, 255, 255, 255});
    }
}

static void render_top_banner(SDL_Renderer *renderer, CockpitState *state) {
    SDL_Rect banner = {0, 0, WINDOW_WIDTH, 56};
    SDL_SetRenderDrawColor(renderer, 13, 17, 23, 255);
    SDL_RenderFillRect(renderer, &banner);
    SDL_SetRenderDrawColor(renderer, 48, 54, 61, 255);
    SDL_RenderDrawLine(renderer, 0, 56, WINDOW_WIDTH, 56);

    // State Pill Box
    SDL_Rect pill = {20, 12, 190, 32};
    const char *state_txt = "DISARMED (STANDBY)";
    SDL_Color pill_color = {70, 80, 95, 255};

    if (state->fast.state == STATE_ARMED || state->fast.state == STATE_FLIGHT_NORMAL) {
        state_txt = "ARMED (FLIGHT NORMAL)";
        pill_color = (SDL_Color){0, 180, 80, 255};
    } else if (state->fast.state == STATE_FAILSAFE_ACTIVE) {
        state_txt = "! FAILSAFE ACTIVE !";
        pill_color = (SDL_Color){220, 30, 30, 255};
    }
    SDL_SetRenderDrawColor(renderer, pill_color.r, pill_color.g, pill_color.b, 255);
    SDL_RenderFillRect(renderer, &pill);
    draw_string(renderer, state_txt, 28, 23, 1, (SDL_Color){255, 255, 255, 255});

    // Connection Pill
    SDL_Rect conn_pill = {225, 12, 210, 32};
    const char *conn_txt = "HIL SIMULATION (STANDALONE)";
    SDL_Color conn_col = {150, 100, 30, 255};

    if (state->is_hardware_connected) {
        conn_txt = "PICO 2 W (RP2350 USB CDC)";
        conn_col = (SDL_Color){0, 120, 210, 255};
    }
    SDL_SetRenderDrawColor(renderer, conn_col.r, conn_col.g, conn_col.b, 255);
    SDL_RenderFillRect(renderer, &conn_pill);
    draw_string(renderer, conn_txt, 232, 23, 1, (SDL_Color){255, 255, 255, 255});

    // Diagnostic Stats
    char stats1[64], stats2[64];
    snprintf(stats1, sizeof(stats1), "RTOS CORE0: %d%% | CORE1: %d%%", state->slow.core0_cpu_pct, state->slow.core1_cpu_pct);
    snprintf(stats2, sizeof(stats2), "HEAP: %u KB | JITTER: %u us", state->slow.free_heap_bytes / 1024, state->slow.loop_jitter_us);
    draw_string(renderer, stats1, 460, 16, 1, (SDL_Color){160, 180, 205, 255});
    draw_string(renderer, stats2, 460, 32, 1, (SDL_Color){130, 150, 175, 255});

    // Safety Watchdog Status
    draw_string(renderer, "WDT: ARMED (100ms)", WINDOW_WIDTH - 190, 23, 1, (SDL_Color){0, 230, 118, 255});
}

void cockpit_render(SDL_Renderer *renderer, CockpitState *state) {
    SDL_SetRenderDrawColor(renderer, 10, 14, 20, 255);
    SDL_RenderClear(renderer);

    render_top_banner(renderer, state);
    render_artificial_horizon(renderer, state, 205, 305, 145);
    render_strip_chart(renderer, state, 420, 68, 584, 230);
    render_motor_gauges(renderer, state, 420, 312, 584, 215);

    // Bottom Control / Fault Deck
    SDL_Rect deck = {15, 542, WINDOW_WIDTH - 30, 64};
    SDL_SetRenderDrawColor(renderer, 18, 24, 32, 255);
    SDL_RenderFillRect(renderer, &deck);
    SDL_SetRenderDrawColor(renderer, 48, 60, 76, 255);
    SDL_RenderDrawRect(renderer, &deck);

    draw_string(renderer, "PILOT COMMAND & FAULT INJECTION DECK:", 30, 552, 1, (SDL_Color){255, 214, 0, 255});

    const char *active_fault_label = "FAULT: NONE (NOMINAL)";
    SDL_Color fcol = {0, 230, 118, 255};
    if (state->injected_fault == FAULT_GYRO_BIAS) {
        active_fault_label = "FAULT: GYRO DRIFT INJECTED (FAILSAFE TRIGGERED)";
        fcol = (SDL_Color){255, 60, 60, 255};
    } else if (state->injected_fault == FAULT_MOTOR_3_CUT) {
        active_fault_label = "FAULT: MOTOR 3 CUTOUT (FAILSAFE ACTIVE)";
        fcol = (SDL_Color){255, 60, 60, 255};
    } else if (state->injected_fault == FAULT_LINK_TIMEOUT) {
        active_fault_label = "FAULT: TELEMETRY DROPOUT (WDT TIMEOUT)";
        fcol = (SDL_Color){255, 60, 60, 255};
    }
    draw_string(renderer, active_fault_label, 380, 552, 1, fcol);

    // Hotkey Buttons Guide
    draw_string(renderer, "[SPACE/A] ARM   [ARROWS/W,S,Q,E] TILT PITCH/ROLL   [1] GYRO FAULT   [2] CUT MOTOR   [R] RESET", 20, 580, 1, (SDL_Color){170, 195, 220, 255});

    SDL_RenderPresent(renderer);
}

void cockpit_inject_fault(CockpitState *state, HilFaultType fault) {
    state->injected_fault = fault;
    state->fault_start_time = SDL_GetTicks();
}

void cockpit_toggle_arm(CockpitState *state) {
    if (state->fast.state == STATE_STANDBY_DISARMED) {
        state->fast.state = STATE_ARMED;
    } else {
        state->fast.state = STATE_STANDBY_DISARMED;
    }
}
