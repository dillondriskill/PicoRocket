cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(rocket C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(rocket
  src/rocket/rocket.c
  src/kernel/kernel.c
  src/terminal/terminal.c
  src/guidance/guidance.c
)

pico_enable_stdio_usb(rocket 1)
pico_enable_stdio_uart(rocket 1)

pico_add_extra_outputs(rocket)

target_link_libraries(rocket pico_stdlib hardware_gpio hardware_watchdog pico_bootrom)
