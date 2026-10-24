#ifndef PHYSICS_SIM_H
#define PHYSICS_SIM_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Rigid body state
    float roll_deg;      // phi
    float pitch_deg;     // theta
    float yaw_deg;       // psi
    
    float p_dps;         // Roll rate (deg/s)
    float q_dps;         // Pitch rate (deg/s)
    float r_dps;         // Yaw rate (deg/s)
    
    float altitude_m;    // Altitude above ground (meters)
    float velocity_z_ms; // Vertical speed (m/s)
    
    // Physical vehicle constants
    float mass_kg;       // Vehicle mass (e.g., 0.85 kg)
    float arm_length_m;  // Motor-to-center distance (0.125 m)
    float Ixx, Iyy, Izz; // Moments of inertia (kg*m^2)
    float thrust_coeff;  // Thrust per PWM unit (N / unit)
    
    // Fault injection parameters
    HilFaultType active_fault;
    float gyro_drift_deg;
    float motor3_health_factor; // 1.0 = nominal, 0.0 = completely failed
} PhysicsSimulator6DOF;

void physics_sim_init(PhysicsSimulator6DOF *sim);
void physics_sim_step(PhysicsSimulator6DOF *sim, const uint16_t motor_pwm[4], float dt);
void physics_sim_inject_fault(PhysicsSimulator6DOF *sim, HilFaultType fault);
void physics_sim_reset(PhysicsSimulator6DOF *sim);

#ifdef __cplusplus
}
#endif

#endif // PHYSICS_SIM_H
