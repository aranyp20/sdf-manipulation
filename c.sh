#!/bin/sh
set -e
cd "$(dirname "$0")"

if [ ! -d build ]; then
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi
cmake --build build -j
ln -sf build/compile_commands.json compile_commands.json
