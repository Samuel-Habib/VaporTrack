#!/bin/bash
set -e

# 1. Clean previous build artifacts
rm -rf build/

# 2. Configure CMake
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug -B build

# 3. Determine core count (portable for Linux & macOS)
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif command -v sysctl >/dev/null 2>&1 && sysctl -n hw.logicalcpu >/dev/null 2>&1; then
    JOBS=$(sysctl -n hw.logicalcpu)
else
    JOBS=4
fi

# 4. Build the project
cmake --build build -j"$JOBS"

# 5. Display memory usage
arm-none-eabi-size build/vapor_track.elf

# 6. Optional: Flash the binary (uncomment if hardware connected)
# st-flash --reset write build/vapor_track.bin 0x08000000
