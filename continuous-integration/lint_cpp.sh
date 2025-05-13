#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/..")

cppFiles=$(find "$ROOT_DIR" -name '*.cpp')
hFiles=$(find "$ROOT_DIR" -name '*.h')

if [ -z "$cppFiles$hFiles" ]; then
    echo Error: Cannot find C++ source files
    exit 1
fi

echo "Formatting C++ source files with clang-format"
clang-format --version || exit $?
for fileName in $cppFiles $hFiles; do
    echo "Formatting $fileName"
    clang-format -i $fileName || exit $?
done

echo Success! ["$(basename $0)"]
