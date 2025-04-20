# invoke using: mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 && cd ..

cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED ENV{PICO_SDK_PATH})
  set(ENV{PICO_SDK_PATH} "~/Downloads/pico-sdk/")
endif()
if (NOT DEFINED ENV{PICO_TOOLCHAIN_PATH})
  set(ENV{PICO_TOOLCHAIN_PATH} "~/Downloads/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin")
endif()

# initialize the SDK directly
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(picoadc
    picoadc.c
    fft_anywhere.c
)

# add some warning flags
set("CMAKE_C_FLAGS" "${CMAKE_C_FLAGS}  -Wall -Wextra -Wshadow")

# add some things to the cflags to make floating point math faster without breaking it
# without these flags, complex arithmetic has no choice but to follow strict ieee 754 rules
# which make them insanely slow and power hungry for no practical benefit
set("CMAKE_C_FLAGS" "${CMAKE_C_FLAGS} -O3 -ffinite-math-only -fno-signed-zeros -fno-rounding-math -fno-trapping-math -fno-math-errno -fcx-limited-range")

# pull in common dependencies
target_link_libraries(picoadc pico_stdlib cmsis_core hardware_adc hardware_dma hardware_pwm)

pico_enable_stdio_usb(picoadc 1)
target_compile_definitions(picoadc PRIVATE PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=-1 )

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(picoadc)
