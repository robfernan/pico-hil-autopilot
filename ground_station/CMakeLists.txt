cmake_minimum_required(VERSION 3.16)
project(aero_ground_station C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

include_directories(
    include
    ${SDL2_INCLUDE_DIRS}
)

add_executable(aero_ground_station
    src/main.c
    src/cockpit.c
    src/serial_bridge.c
)

target_link_libraries(aero_ground_station
    PRIVATE
    ${SDL2_LIBRARIES}
    m
    pthread
)

target_compile_options(aero_ground_station PRIVATE -Wall -Wextra -O2)
