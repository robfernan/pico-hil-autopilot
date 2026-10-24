#include "physics_sim.h"
#include <math.h>

#define DEG_TO_RAD (3.1415926535f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.1415926535f)
#define GRAVITY_MSS 9.80665f

void physics_sim_init(PhysicsSimulator6DOF *sim) {
    sim->roll_deg = 0.0f;
    sim->pitch_deg = 0.0f;
    sim->yaw_deg = 0.0f;
    
    sim->p_dps = 0.0f;
    sim->q_dps = 0.0f;
    sim->r_dps = 0.0f;
    
    sim->altitude_m = 0.0f;
    sim->velocity_z_ms = 0.0f;
    
    // Vehicle specs (250mm quadrotor)
    sim->mass_kg = 0.75f;
    sim->arm_length_m = 0.12f;
    sim->Ixx = 0.008f;
    sim->Iyy = 0.008f;
    sim->Izz = 0.015f;
    sim->thrust_coeff = 0.0035f; // Newtons per PWM unit
    
    sim->active_fault = FAULT_NONE;
    sim->gyro_drift_deg = 0.0f;
    sim->motor3_health_factor = 1.0f;
}

void physics_sim_step(PhysicsSimulator6DOF *sim, const uint16_t motor_pwm[4], float dt) {
    if (dt <= 0.0f) return;

    // Apply motor faults
    float m1 = (float)motor_pwm[0] * sim->thrust_coeff;
    float m2 = (float)motor_pwm[1] * sim->thrust_coeff;
    float m3 = (float)motor_pwm[2] * sim->thrust_coeff * sim->motor3_health_factor;
    float m4 = (float)motor_pwm[3] * sim->thrust_coeff;

    // Total vertical thrust (body frame)
    float total_thrust_N = m1 + m2 + m3 + m4;

    // Torques:
    // M1: Front-Left (CW)
    // M2: Front-Right (CCW)
    // M3: Rear-Left (CCW)
    // M4: Rear-Right (CW)
    float d = sim->arm_length_m * 0.7071f; // 45 degree arms
    float torque_roll  = d * ((m2 + m4) - (m1 + m3));
    float torque_pitch = d * ((m1 + m2) - (m3 + m4));
    float torque_yaw   = 0.02f * ((m1 + m4) - (m2 + m3)); // differential drag torque

    // Angular accelerations: alpha = tau / I - damping
    float damping = 0.15f;
    float alpha_p = (torque_roll  / sim->Ixx) * RAD_TO_DEG - damping * sim->p_dps;
    float alpha_q = (torque_pitch / sim->Iyy) * RAD_TO_DEG - damping * sim->q_dps;
    float alpha_r = (torque_yaw   / sim->Izz) * RAD_TO_DEG - damping * sim->r_dps;

    // Integrate angular velocities
    sim->p_dps += alpha_p * dt;
    sim->q_dps += alpha_q * dt;
    sim->r_dps += alpha_r * dt;

    // Euler angle kinematic rates
    float phi   = sim->roll_deg * DEG_TO_RAD;
    float theta = sim->pitch_deg * DEG_TO_RAD;

    float roll_dot  = sim->p_dps + (sim->q_dps * sinf(phi) + sim->r_dps * cosf(phi)) * tanf(theta);
    float pitch_dot = sim->q_dps * cosf(phi) - sim->r_dps * sinf(phi);
    float yaw_dot   = (sim->q_dps * sinf(phi) + sim->r_dps * cosf(phi)) / (cosf(theta) + 0.0001f);

    sim->roll_deg  += roll_dot * dt;
    sim->pitch_deg += pitch_dot * dt;
    sim->yaw_deg   += yaw_dot * dt;

    // Wrap yaw into [0, 360)
    while (sim->yaw_deg < 0.0f)   sim->yaw_deg += 360.0f;
    while (sim->yaw_deg >= 360.0f) sim->yaw_deg -= 360.0f;

    // Altitude acceleration in inertial Z
    float acc_z = (total_thrust_N * cosf(phi) * cosf(theta) / sim->mass_kg) - GRAVITY_MSS;
    sim->velocity_z_ms += acc_z * dt;
    sim->altitude_m += sim->velocity_z_ms * dt;

    // Ground contact constraint
    if (sim->altitude_m <= 0.0f) {
        sim->altitude_m = 0.0f;
        if (sim->velocity_z_ms < 0.0f) sim->velocity_z_ms = 0.0f;
    }
}

void physics_sim_inject_fault(PhysicsSimulator6DOF *sim, HilFaultType fault) {
    sim->active_fault = fault;
    if (fault == FAULT_MOTOR_3_CUT) {
        sim->motor3_health_factor = 0.0f; // completely dead motor 3
    } else {
        sim->motor3_health_factor = 1.0f;
    }

    if (fault == FAULT_GYRO_BIAS) {
        sim->gyro_drift_deg = 20.0f;
    } else {
        sim->gyro_drift_deg = 0.0f;
    }
}

void physics_sim_reset(PhysicsSimulator6DOF *sim) {
    physics_sim_init(sim);
}
