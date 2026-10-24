#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>
#include "cockpit.h"

typedef struct {
    int fd;
    char port[64];
    bool is_open;
    bool should_run;
    pthread_t thread;
    pthread_mutex_t mutex;
    CockpitState *cockpit;
} SerialBridge;

bool serial_bridge_init(SerialBridge *bridge, CockpitState *cockpit, const char *port);
void serial_bridge_close(SerialBridge *bridge);
bool serial_bridge_send(SerialBridge *bridge, uint8_t msg_id, const void *payload, uint8_t len);

#endif // SERIAL_BRIDGE_H
