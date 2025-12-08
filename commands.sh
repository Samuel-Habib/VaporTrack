#!/bin/bash

rm -rf build/

st-flash erase

cmake -DCMAKE_TOOLCHAIN_FILE=stm32_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug -B build

cmake --build build -j$(nproc)

arm-none-eabi-size build/vapor_track

arm-none-eabi-objcopy -O binary build/vapor_track build/vapor_track.bin

st-flash --reset write build/vapor_track.bin 0x08000000
