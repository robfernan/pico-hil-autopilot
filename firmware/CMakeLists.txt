cmake_minimum_required(VERSION 3.16)

# Check if compiling with Pico SDK for RP2350 target
if(DEFINED ENV{PICO_SDK_PATH} OR DEFINED PICO_SDK_PATH)
    include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
    project(pico_hil_autopilot C CXX ASM)
    pico_sdk_init()

    add_executable(pico_hil_autopilot
        src/main.c
        src/state_machine.c
        src/pid_controller.c
        src/physics_sim.c
        src/hil_engine.c
    )

    target_include_directories(pico_hil_autopilot PRIVATE include)

    target_link_libraries(pico_hil_autopilot
        pico_stdlib
        pico_multicore
        hardware_watchdog
        hardware_timer
    )

    pico_enable_stdio_usb(pico_hil_autopilot 1)
    pico_enable_stdio_uart(pico_hil_autopilot 0)

    pico_add_extra_outputs(pico_hil_autopilot)

else()
    # Host simulation & unit-test target
    project(pico_hil_autopilot_host C)
    set(CMAKE_C_STANDARD 11)

    include_directories(include)

    add_executable(hil_firmware_sim
        src/main.c
        src/state_machine.c
        src/pid_controller.c
        src/physics_sim.c
        src/hil_engine.c
    )

    target_link_libraries(hil_firmware_sim PRIVATE m)
    target_compile_options(hil_firmware_sim PRIVATE -Wall -Wextra -O2)
endif()
