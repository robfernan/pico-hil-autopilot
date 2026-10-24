#include <stdio.h>
#include <string.h>
#include "protocol.h"
#include "hil_engine.h"

#ifdef PICO_BOARD
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#else
#include <unistd.h>
#include <time.h>
#endif

static HilEngine g_engine;

// USB Packet Serializer
static void send_telemetry_packet(uint8_t msg_id, const void *payload, uint8_t len) {
    uint8_t header[4] = {
        PROTOCOL_MAGIC_0,
        PROTOCOL_MAGIC_1,
        msg_id,
        len
    };
    uint16_t crc = protocol_crc16((const uint8_t *)payload, len);
    uint8_t crc_bytes[2] = {
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };

    fwrite(header, 1, 4, stdout);
    if (len > 0 && payload != NULL) {
        fwrite(payload, 1, len, stdout);
    }
    fwrite(crc_bytes, 1, 2, stdout);
    fflush(stdout);
}

// Ingest binary packet from USB serial
static void process_incoming_usb(void) {
#ifdef PICO_BOARD
    static uint8_t rx_state = 0;
    static uint8_t msg_id = 0;
    static uint8_t len = 0;
    static uint8_t idx = 0;
    static uint8_t payload[64];
    static uint8_t crc_bytes[2];

    while (1) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;

        uint8_t b = (uint8_t)c;
        switch (rx_state) {
            case 0: // Seek Magic 0
                if (b == PROTOCOL_MAGIC_0) rx_state = 1;
                else if (b == 'A' || b == 'a') {
                    hil_engine_handle_command(&g_engine, MSG_CMD_ARM, NULL, 0);
                } else if (b == 'D' || b == 'd' || b == 'R' || b == 'r') {
                    hil_engine_handle_command(&g_engine, MSG_CMD_DISARM, NULL, 0);
                    physics_sim_inject_fault(&g_engine.sim, FAULT_NONE);
                } else if (b == '1') {
                    physics_sim_inject_fault(&g_engine.sim, FAULT_GYRO_BIAS);
                } else if (b == '2') {
                    physics_sim_inject_fault(&g_engine.sim, FAULT_MOTOR_3_CUT);
                }
                break;

            case 1: // Seek Magic 1
                if (b == PROTOCOL_MAGIC_1) rx_state = 2;
                else rx_state = 0;
                break;

            case 2: // Msg ID
                msg_id = b;
                rx_state = 3;
                break;

            case 3: // Payload Length
                len = b;
                idx = 0;
                if (len > sizeof(payload)) rx_state = 0;
                else if (len == 0) rx_state = 5;
                else rx_state = 4;
                break;

            case 4: // Payload Data
                payload[idx++] = b;
                if (idx >= len) {
                    idx = 0;
                    rx_state = 5;
                }
                break;

            case 5: // CRC Bytes
                crc_bytes[idx++] = b;
                if (idx >= 2) {
                    uint16_t expected_crc = ((uint16_t)crc_bytes[0] << 8) | crc_bytes[1];
                    uint16_t computed_crc = protocol_crc16(payload, len);
                    if (expected_crc == computed_crc) {
                        hil_engine_handle_command(&g_engine, msg_id, payload, len);
                    }
                    rx_state = 0;
                }
                break;
        }
    }
#endif
}

// Core 1 Entrypoint: Communication, Telemetry Streaming & Safety Supervisor
void core1_entry(void) {
    uint32_t last_fast_tx = 0;
    uint32_t last_slow_tx = 0;

    while (1) {
#ifdef PICO_BOARD
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        watchdog_update();
#else
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint32_t now_ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif

        // Process incoming commands from USB CDC
        process_incoming_usb();

        // 100 Hz Fast Telemetry
        if (now_ms - last_fast_tx >= 10) {
            last_fast_tx = now_ms;
            send_telemetry_packet(MSG_TELEMETRY_FAST, &g_engine.fast_packet, sizeof(TelemetryFastPacket));
        }

        // 10 Hz Slow System Telemetry
        if (now_ms - last_slow_tx >= 100) {
            last_slow_tx = now_ms;
            send_telemetry_packet(MSG_TELEMETRY_SLOW, &g_engine.slow_packet, sizeof(TelemetrySlowPacket));
        }

#ifdef PICO_BOARD
        sleep_us(500);
#else
        usleep(1000);
#endif
    }
}

// Core 0 Entrypoint: 500 Hz Real-Time Dynamics & Closed-Loop Control
int main(void) {
#ifdef PICO_BOARD
    stdio_init_all();
    
    // Enable RP2350 hardware watchdog (500ms timeout)
    if (watchdog_caused_reboot()) {
        printf("[WARN] RP2350 rebooted via hardware watchdog!\n");
    }
    watchdog_enable(500, 1);
#endif

    hil_engine_init(&g_engine);

#ifdef PICO_BOARD
    // Launch Core 1 telemetry supervisor
    multicore_launch_core1(core1_entry);
#endif

    uint32_t last_control_tick = 0;
    const float dt = 0.002f; // 500 Hz (2.0 ms)

    while (1) {
#ifdef PICO_BOARD
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
#else
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint32_t now_ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif

        if (now_ms != last_control_tick) {
            last_control_tick = now_ms;
            hil_engine_core0_step(&g_engine, dt, now_ms);
        }

#ifdef PICO_BOARD
        sleep_us(1000);
#else
        usleep(2000);
        process_incoming_usb();
        static uint32_t host_tx = 0;
        if (now_ms - host_tx >= 10) {
            host_tx = now_ms;
            send_telemetry_packet(MSG_TELEMETRY_FAST, &g_engine.fast_packet, sizeof(TelemetryFastPacket));
        }
#endif
    }
    return 0;
}
