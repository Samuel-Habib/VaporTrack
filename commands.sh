#!/bin/bash

# 1. Clean previous build artifacts
rm -rf build/

# 2. Erase the chip
# Note: If this fails with "Failed to reset", hold the physical RESET button 
# on the board, run the script, and release when it says "Mass erasing".
st-flash erase

# 3. Configure CMake with the correct toolchain path for macOS
# Pointing to the file actually present in your /cmake directory
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -B build

# 4. Build the project
# Replaced 'nproc' (Linux) with 'sysctl' (macOS) to fix the core count error
cmake --build build -j$(sysctl -n hw.logicalcpu)

# 5. Display memory usage
arm-none-eabi-size build/vapor_track.elf

# 6. Convert ELF to Binary
arm-none-eabi-objcopy -O binary build/vapor_track.elf build/vapor_track.bin

# 7. Flash the binary to the board
st-flash --reset write build/vapor_track.bin 0x08000000
