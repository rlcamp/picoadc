#!/bin/bash
# this script should ideally work for any pico-sdk based repositories

set -e

PICO_TOOLCHAIN_PATH=$(dirname $((find ${HOME}/Downloads -name arm-none-eabi-gcc -maxdepth 3 -mindepth 3 | sort; find /Applications/ArmGNUToolchain -name arm-none-eabi-gcc 2>/dev/null | sort) | tail -n1))

PICO_SDK_PATH=$(dirname $(find ${HOME}/Downloads -name pico_sdk_init.cmake -mindepth 2 -maxdepth 2))

set -x

mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 -DPICO_SDK_PATH="$PICO_SDK_PATH" -DPICO_TOOLCHAIN_PATH="$PICO_TOOLCHAIN_PATH" && cmake ..
