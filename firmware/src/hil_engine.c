#include "hil_engine.h"
#include <string.h>

void hil_engine_init(HilEngine *engine) {
    memset(engine, 0, sizeof(HilEngine));
    
    physics_sim_init(&engine->sim);
    state_machine_init(&engine->sm);

    // Initialize Cascaded PID loops:
    // Outer loop: Angle error -> target angular rate (deg/s)
    pid_init(&engine->pid_roll_angle, 4.5f, 0.05f, 0.0f, 50.0f, 200.0f, 0.8f);
    pid_init(&engine->pid_pitch_angle, 4.5f, 0.05f, 0.0f, 50.0f, 200.0f, 0.8f);

    // Inner loop: Angular rate error -> differential PWM
    pid_init(&engine->pid_roll_rate, 1.2f, 0.2f, 0.04f, 80.0f, 300.0f, 0.6f);
    pid_init(&engine->pid_pitch_rate, 1.2f, 0.2f, 0.04f, 80.0f, 300.0f, 0.6f);
    pid_init(&engine->pid_yaw_rate, 2.0f, 0.1f, 0.0f, 50.0f, 200.0f, 0.8f);

    engine->target_roll = 0.0f;
    engine->target_pitch = 0.0f;
    engine->target_yaw = 0.0f;
    engine->target_altitude = 1.5f;

    engine->slow_packet.core0_cpu_pct = 48;
    engine->slow_packet.core1_cpu_pct = 16;
    engine->slow_packet.free_heap_bytes = 204800;
    engine->slow_packet.loop_jitter_us = 3;
}

void hil_engine_core0_step(HilEngine *engine, float dt, uint32_t now_ms) {
    if (dt <= 0.0f) return;
    engine->loop_counter++;

    // Step the 6-DOF dynamic physics solver
    physics_sim_step(&engine->sim, engine->motor_pwm, dt);

    // Synthesize IMU sensor readouts with simulated drift/noise
    float measured_roll  = engine->sim.roll_deg + engine->sim.gyro_drift_deg;
    float measured_pitch = engine->sim.pitch_deg;
    float measured_yaw   = engine->sim.yaw_deg;

    float measured_p = engine->sim.p_dps;
    float measured_q = engine->sim.q_dps;
    float measured_r = engine->sim.r_dps;

    // Update safety state machine
    state_machine_update(&engine->sm, measured_roll, measured_pitch, 
                         measured_p, measured_q, measured_r, now_ms);

    if (state_machine_can_spin_motors(&engine->sm)) {
        // Outer loop (Angle Error -> Target Rate)
        float desired_p = pid_update(&engine->pid_roll_angle, engine->target_roll - measured_roll, dt);
        float desired_q = pid_update(&engine->pid_pitch_angle, engine->target_pitch - measured_pitch, dt);

        // Inner loop (Rate Error -> Differential Torque Command)
        float u_roll  = pid_update(&engine->pid_roll_rate, desired_p - measured_p, dt);
        float u_pitch = pid_update(&engine->pid_pitch_rate, desired_q - measured_q, dt);
        float u_yaw   = pid_update(&engine->pid_yaw_rate, 0.0f - measured_r, dt);

        // Base hover throttle
        float base_thrust = 560.0f;
        if (engine->sm.current_state == STATE_FAILSAFE_ACTIVE) {
            base_thrust = 350.0f; // Controlled descent
        }

        // Mixer: Quad-X layout
        float m1 = base_thrust + u_pitch - u_roll + u_yaw;
        float m2 = base_thrust + u_pitch + u_roll - u_yaw;
        float m3 = base_thrust - u_pitch - u_roll - u_yaw;
        float m4 = base_thrust - u_pitch + u_roll + u_yaw;

        // Clamp to valid PWM duty range [0, 1000]
        float motors[4] = {m1, m2, m3, m4};
        for (int i = 0; i < 4; ++i) {
            if (motors[i] < 0.0f) motors[i] = 0.0f;
            if (motors[i] > 1000.0f) motors[i] = 1000.0f;
            engine->motor_pwm[i] = (uint16_t)motors[i];
        }
    } else {
        // Disarmed or Emergency Kill -> zero outputs
        engine->motor_pwm[0] = 0;
        engine->motor_pwm[1] = 0;
        engine->motor_pwm[2] = 0;
        engine->motor_pwm[3] = 0;
        pid_reset(&engine->pid_roll_angle);
        pid_reset(&engine->pid_roll_rate);
        pid_reset(&engine->pid_pitch_angle);
        pid_reset(&engine->pid_pitch_rate);
        pid_reset(&engine->pid_yaw_rate);
    }

    // Populate telemetry snapshot for Core 1
    engine->fast_packet.timestamp_ms = now_ms;
    engine->fast_packet.roll = (int16_t)(measured_roll * 100.0f);
    engine->fast_packet.pitch = (int16_t)(measured_pitch * 100.0f);
    engine->fast_packet.yaw = (int16_t)(measured_yaw * 100.0f);
    engine->fast_packet.gyro_x = (int16_t)(measured_p * 10.0f);
    engine->fast_packet.gyro_y = (int16_t)(measured_q * 10.0f);
    engine->fast_packet.gyro_z = (int16_t)(measured_r * 10.0f);
    engine->fast_packet.altitude_cm = (int32_t)(engine->sim.altitude_m * 100.0f);
    engine->fast_packet.velocity_z_cms = (int16_t)(engine->sim.velocity_z_ms * 100.0f);
    engine->fast_packet.motor_pwm[0] = engine->motor_pwm[0];
    engine->fast_packet.motor_pwm[1] = engine->motor_pwm[1];
    engine->fast_packet.motor_pwm[2] = engine->motor_pwm[2];
    engine->fast_packet.motor_pwm[3] = engine->motor_pwm[3];
    engine->fast_packet.state = (uint8_t)engine->sm.current_state;
    engine->fast_packet.active_fault = (uint8_t)engine->sim.active_fault;
}

void hil_engine_handle_command(HilEngine *engine, uint8_t msg_id, const uint8_t *payload, uint8_t len) {
    switch (msg_id) {
        case MSG_CMD_ARM:
            state_machine_request_arm(&engine->sm, true);
            break;

        case MSG_CMD_DISARM:
            state_machine_request_arm(&engine->sm, false);
            break;

        case MSG_CMD_INJECT_FAULT:
            if (len >= sizeof(CmdInjectFaultPacket)) {
                const CmdInjectFaultPacket *cmd = (const CmdInjectFaultPacket *)payload;
                physics_sim_inject_fault(&engine->sim, (HilFaultType)cmd->fault_type);
            }
            break;

        case MSG_CMD_SET_PID:
            if (len >= sizeof(CmdSetPidPacket)) {
                const CmdSetPidPacket *cmd = (const CmdSetPidPacket *)payload;
                if (cmd->axis == 0) {
                    pid_set_gains(&engine->pid_roll_angle, cmd->kp, cmd->ki, cmd->kd);
                } else if (cmd->axis == 1) {
                    pid_set_gains(&engine->pid_pitch_angle, cmd->kp, cmd->ki, cmd->kd);
                }
            }
            break;

        case MSG_CMD_SETPOINT:
            if (len >= sizeof(CmdSetpointPacket)) {
                const CmdSetpointPacket *cmd = (const CmdSetpointPacket *)payload;
                engine->target_pitch = (float)cmd->target_pitch / 100.0f;
                engine->target_roll  = (float)cmd->target_roll / 100.0f;
                engine->target_yaw   = (float)cmd->target_yaw / 100.0f;
            }
            break;

        default:
            break;
    }
}
