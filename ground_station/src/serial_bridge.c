#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include "serial_bridge.h"
#include "protocol.h"

static int configure_serial(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // read doesn't block
    tty.c_cc[VTIME] = 1;                        // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return 0;
}

typedef enum {
    RX_SEEK_MAGIC_0,
    RX_SEEK_MAGIC_1,
    RX_READ_HEADER,
    RX_READ_PAYLOAD,
    RX_READ_CRC
} RxParserState;

static void *serial_rx_thread(void *arg) {
    SerialBridge *bridge = (SerialBridge *)arg;
    uint8_t buffer[256];
    
    RxParserState state = RX_SEEK_MAGIC_0;
    uint8_t current_msg_id = 0;
    uint8_t payload_len = 0;
    uint8_t payload_idx = 0;
    uint8_t payload_buf[128];
    uint16_t received_crc = 0;
    uint8_t crc_bytes[2];

    while (bridge->should_run) {
        if (!bridge->is_open) {
            int fd = open(bridge->port, O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd >= 0) {
                if (configure_serial(fd, B115200) == 0) {
                    pthread_mutex_lock(&bridge->mutex);
                    bridge->fd = fd;
                    bridge->is_open = true;
                    bridge->cockpit->is_hardware_connected = true;
                    pthread_mutex_unlock(&bridge->mutex);
                    printf("[SERIAL] Connected to Pico 2 W on %s\n", bridge->port);
                } else {
                    close(fd);
                }
            }
            if (!bridge->is_open) {
                usleep(500000); // retry every 500ms
                continue;
            }
        }

        int bytes_read = read(bridge->fd, buffer, sizeof(buffer));
        if (bytes_read <= 0) {
            if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                // Connection broken
                pthread_mutex_lock(&bridge->mutex);
                close(bridge->fd);
                bridge->fd = -1;
                bridge->is_open = false;
                bridge->cockpit->is_hardware_connected = false;
                pthread_mutex_unlock(&bridge->mutex);
                printf("[SERIAL] Pico disconnected from %s\n", bridge->port);
            }
            usleep(2000);
            continue;
        }

        for (int i = 0; i < bytes_read; ++i) {
            uint8_t b = buffer[i];

            switch (state) {
                case RX_SEEK_MAGIC_0:
                    if (b == PROTOCOL_MAGIC_0) state = RX_SEEK_MAGIC_1;
                    break;

                case RX_SEEK_MAGIC_1:
                    if (b == PROTOCOL_MAGIC_1) {
                        state = RX_READ_HEADER;
                        payload_idx = 0;
                    } else {
                        state = RX_SEEK_MAGIC_0;
                    }
                    break;

                case RX_READ_HEADER:
                    if (payload_idx == 0) {
                        current_msg_id = b;
                        payload_idx++;
                    } else {
                        payload_len = b;
                        payload_idx = 0;
                        if (payload_len > sizeof(payload_buf)) {
                            state = RX_SEEK_MAGIC_0; // Invalid packet size
                        } else if (payload_len == 0) {
                            state = RX_READ_CRC;
                        } else {
                            state = RX_READ_PAYLOAD;
                        }
                    }
                    break;

                case RX_READ_PAYLOAD:
                    payload_buf[payload_idx++] = b;
                    if (payload_idx >= payload_len) {
                        payload_idx = 0;
                        state = RX_READ_CRC;
                    }
                    break;

                case RX_READ_CRC:
                    crc_bytes[payload_idx++] = b;
                    if (payload_idx >= 2) {
                        received_crc = ((uint16_t)crc_bytes[0] << 8) | crc_bytes[1];
                        
                        // Validate CRC
                        uint16_t computed_crc = protocol_crc16(payload_buf, payload_len);
                        pthread_mutex_lock(&bridge->mutex);
                        if (computed_crc == received_crc) {
                            bridge->cockpit->packets_received++;
                            bridge->cockpit->last_packet_time = SDL_GetTicks();
                            
                            if (current_msg_id == MSG_TELEMETRY_FAST && payload_len == sizeof(TelemetryFastPacket)) {
                                memcpy(&bridge->cockpit->fast, payload_buf, sizeof(TelemetryFastPacket));
                            } else if (current_msg_id == MSG_TELEMETRY_SLOW && payload_len == sizeof(TelemetrySlowPacket)) {
                                memcpy(&bridge->cockpit->slow, payload_buf, sizeof(TelemetrySlowPacket));
                            }
                        } else {
                            bridge->cockpit->crc_errors++;
                        }
                        pthread_mutex_unlock(&bridge->mutex);

                        state = RX_SEEK_MAGIC_0;
                    }
                    break;
            }
        }
    }
    return NULL;
}

bool serial_bridge_init(SerialBridge *bridge, CockpitState *cockpit, const char *port) {
    memset(bridge, 0, sizeof(SerialBridge));
    bridge->cockpit = cockpit;
    strncpy(bridge->port, port, sizeof(bridge->port) - 1);
    bridge->fd = -1;
    bridge->should_run = true;
    pthread_mutex_init(&bridge->mutex, NULL);

    if (pthread_create(&bridge->thread, NULL, serial_rx_thread, bridge) != 0) {
        return false;
    }
    return true;
}

void serial_bridge_close(SerialBridge *bridge) {
    if (!bridge->should_run) return;
    bridge->should_run = false;
    pthread_join(bridge->thread, NULL);
    if (bridge->fd >= 0) close(bridge->fd);
    pthread_mutex_destroy(&bridge->mutex);
}

bool serial_bridge_send(SerialBridge *bridge, uint8_t msg_id, const void *payload, uint8_t len) {
    pthread_mutex_lock(&bridge->mutex);
    if (!bridge->is_open || bridge->fd < 0) {
        pthread_mutex_unlock(&bridge->mutex);
        return false;
    }

    uint8_t packet[256];
    packet[0] = PROTOCOL_MAGIC_0;
    packet[1] = PROTOCOL_MAGIC_1;
    packet[2] = msg_id;
    packet[3] = len;
    
    if (len > 0 && payload != NULL) {
        memcpy(&packet[4], payload, len);
    }
    
    uint16_t crc = protocol_crc16((const uint8_t *)payload, len);
    packet[4 + len] = (uint8_t)(crc >> 8);
    packet[5 + len] = (uint8_t)(crc & 0xFF);

    ssize_t written = write(bridge->fd, packet, 6 + len);
    pthread_mutex_unlock(&bridge->mutex);
    return (written == 6 + len);
}
