#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include "cockpit.h"
#include "serial_bridge.h"

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "Failed to initialize SDL2: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "Aero-RTOS Ground Control Station | RP2350 Dual-Core HIL Cockpit",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI
    );

    if (!window) {
        fprintf(stderr, "Failed to create SDL2 window: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (!renderer) {
        fprintf(stderr, "Failed to create SDL2 renderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    CockpitState state;
    cockpit_init(&state);

    SerialBridge bridge;
    serial_bridge_init(&bridge, &state, "/dev/ttyACM0");

    printf("===============================================================\n");
    printf("  AERO-RTOS GROUND CONTROL STATION & HIL SUPERVISOR\n");
    printf("===============================================================\n");
    printf(" Controls:\n");
    printf("  [SPACE / A] : Toggle ARM / DISARM\n");
    printf("  [1]         : Inject Fault: Gyro Bias / Drift\n");
    printf("  [2]         : Inject Fault: Cut Motor 3 (ESC Burnout)\n");
    printf("  [3]         : Inject Fault: Link Dropout / Watchdog\n");
    printf("  [R]         : Reset / Clear Faults\n");
    printf("  [ESC]       : Exit Ground Station\n");
    printf("===============================================================\n");

    bool running = true;
    uint32_t last_time = SDL_GetTicks();

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        running = false;
                        break;
                    case SDLK_SPACE:
                    case SDLK_a:
                        cockpit_toggle_arm(&state);
                        uint8_t cmd_arm = (state.fast.state == STATE_ARMED) ? MSG_CMD_ARM : MSG_CMD_DISARM;
                        serial_bridge_send(&bridge, cmd_arm, NULL, 0);
                        break;
                    case SDLK_1:
                        cockpit_inject_fault(&state, FAULT_GYRO_BIAS);
                        CmdInjectFaultPacket f1 = { FAULT_GYRO_BIAS, 5000 };
                        serial_bridge_send(&bridge, MSG_CMD_INJECT_FAULT, &f1, sizeof(f1));
                        break;
                    case SDLK_2:
                        cockpit_inject_fault(&state, FAULT_MOTOR_3_CUT);
                        CmdInjectFaultPacket f2 = { FAULT_MOTOR_3_CUT, 5000 };
                        serial_bridge_send(&bridge, MSG_CMD_INJECT_FAULT, &f2, sizeof(f2));
                        break;
                    case SDLK_3:
                        cockpit_inject_fault(&state, FAULT_LINK_TIMEOUT);
                        CmdInjectFaultPacket f3 = { FAULT_LINK_TIMEOUT, 3000 };
                        serial_bridge_send(&bridge, MSG_CMD_INJECT_FAULT, &f3, sizeof(f3));
                        break;
                    case SDLK_r:
                        cockpit_inject_fault(&state, FAULT_NONE);
                        state.fast.state = STATE_STANDBY_DISARMED;
                        CmdInjectFaultPacket f0 = { FAULT_NONE, 0 };
                        serial_bridge_send(&bridge, MSG_CMD_INJECT_FAULT, &f0, sizeof(f0));
                        break;
                }
            }
        }

        uint32_t now = SDL_GetTicks();
        float dt = (now - last_time) / 1000.0f;
        last_time = now;

        cockpit_update(&state, dt);
        cockpit_render(renderer, &state);

        SDL_Delay(16); // ~60 FPS cap
    }

    serial_bridge_close(&bridge);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
