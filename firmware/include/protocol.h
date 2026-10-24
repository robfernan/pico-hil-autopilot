#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_MAGIC_0 0xAA
#define PROTOCOL_MAGIC_1 0x55

// Message IDs
typedef enum {
    MSG_HEARTBEAT       = 0x01,
    MSG_TELEMETRY_FAST  = 0x02, // 100 Hz: Attitude, Angular Rates, Motor PWM
    MSG_TELEMETRY_SLOW  = 0x03, // 10 Hz: Battery, CPU Core usage, Heap, Temps
    MSG_CMD_ARM         = 0x10,
    MSG_CMD_DISARM      = 0x11,
    MSG_CMD_SET_PID     = 0x12,
    MSG_CMD_SETPOINT    = 0x13,
    MSG_CMD_INJECT_FAULT= 0x20,
    MSG_ACK             = 0x30
} MessageId;

// Vehicle State Machine States
typedef enum {
    STATE_BOOT          = 0,
    STATE_INITIALIZING  = 1,
    STATE_STANDBY_DISARMED = 2,
    STATE_ARMED         = 3,
    STATE_FLIGHT_NORMAL = 4,
    STATE_FAILSAFE_ACTIVE = 5,
    STATE_EMERGENCY_KILL = 6
} VehicleState;

// Fault Types for HIL Simulation
typedef enum {
    FAULT_NONE          = 0,
    FAULT_GYRO_BIAS     = 1, // Simulates sudden gyro bias/drift
    FAULT_MOTOR_3_CUT   = 2, // Simulates ESC / motor 3 burnout
    FAULT_LINK_TIMEOUT  = 3, // Simulates lost telemetry link
    FAULT_CPU_OVERLOAD  = 4  // Simulates watchdog near-trigger condition
} HilFaultType;

#pragma pack(push, 1)

// Fast Telemetry (100 Hz from RP2350 -> Linux Ground Station)
typedef struct {
    uint32_t timestamp_ms;
    
    // Euler Angles (degrees * 100)
    int16_t roll;       // -180.00 to +180.00 deg
    int16_t pitch;      // -90.00 to +90.00 deg
    int16_t yaw;        // 0.00 to 360.00 deg
    
    // Angular Rates (deg/s * 10)
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    
    // Altitude & Velocity (cm, cm/s)
    int32_t altitude_cm;
    int16_t velocity_z_cms;
    
    // Motor Outputs (0 - 1000 permille)
    uint16_t motor_pwm[4];
    
    uint8_t state;           // VehicleState
    uint8_t active_fault;    // HilFaultType
} TelemetryFastPacket;

// Slow System Telemetry (10 Hz)
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  core0_cpu_pct;
    uint8_t  core1_cpu_pct;
    uint32_t free_heap_bytes;
    uint16_t loop_jitter_us;
    uint16_t rx_packet_count;
    uint16_t rx_crc_errors;
} TelemetrySlowPacket;

// Command: Set PID Gains
typedef struct {
    uint8_t axis; // 0 = Roll, 1 = Pitch, 2 = Yaw
    float kp;
    float ki;
    float kd;
} CmdSetPidPacket;

// Command: Fault Injection
typedef struct {
    uint8_t fault_type;   // HilFaultType
    uint16_t duration_ms; // Duration before auto-clearing (0 = latch)
} CmdInjectFaultPacket;

// Packet Framing
typedef struct {
    uint8_t magic[2];      // PROTOCOL_MAGIC_0, PROTOCOL_MAGIC_1
    uint8_t msg_id;        // MessageId
    uint8_t payload_len;   // Length of payload
} PacketHeader;

#pragma pack(pop)

// Standard CRC-16-CCITT (Polynomial 0x1021, Init 0xFFFF)
static inline uint16_t protocol_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
