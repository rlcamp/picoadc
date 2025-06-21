# invoke using: mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 && cd ..

cmake_minimum_required(VERSION 3.13)

if (NOT DEFINED ENV{PICO_SDK_PATH})
  set(ENV{PICO_SDK_PATH} "~/Downloads/pico-sdk/")
endif()

if (NOT DEFINED PICO_TOOLCHAIN_PATH)
    execute_process(
        COMMAND bash -c "dirname $((find $\{HOME\}/Downloads -name arm-none-eabi-gcc | sort; find /Applications/ArmGNUToolchain -name arm-none-eabi-gcc | sort) | tail -n1)"
        OUTPUT_VARIABLE PICO_TOOLCHAIN_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

# initialize the SDK directly
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# add some warning flags
set("CMAKE_C_FLAGS" "${CMAKE_C_FLAGS}  -Wall -Wextra -Wshadow")

# add some things to the cflags to make floating point math faster without breaking it
# without these flags, complex arithmetic has no choice but to follow strict ieee 754 rules
# which make them insanely slow and power hungry for no practical benefit
set("CMAKE_C_FLAGS" "${CMAKE_C_FLAGS} -O3 -ffinite-math-only -fno-signed-zeros -fno-rounding-math -fno-trapping-math -fno-math-errno -fcx-limited-range")

add_executable(picoadc_lcd
    picoadc.c
    fft_anywhere.c
    rp2_ili9341_scrolling.c
)

add_executable(picoadc_usb
    picoadc.c
    fft_anywhere.c
    rp2350_usb_cdc_serial.c
)

add_executable(picoadc_lcd_usb
    picoadc.c
    fft_anywhere.c
    rp2_ili9341_scrolling.c
    rp2350_usb_cdc_serial.c
)

target_compile_definitions(picoadc_lcd PUBLIC ENABLE_LCD)
target_compile_definitions(picoadc_usb PUBLIC ENABLE_USB)
target_compile_definitions(picoadc_lcd_usb PUBLIC ENABLE_USB ENABLE_LCD)

target_link_libraries(picoadc_lcd pico_stdlib cmsis_core hardware_adc hardware_dma hardware_spi)
target_link_libraries(picoadc_usb pico_stdlib cmsis_core hardware_adc hardware_dma)
target_link_libraries(picoadc_lcd_usb pico_stdlib cmsis_core hardware_adc hardware_dma hardware_spi)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(picoadc_lcd)
pico_add_extra_outputs(picoadc_usb)
pico_add_extra_outputs(picoadc_lcd_usb)
