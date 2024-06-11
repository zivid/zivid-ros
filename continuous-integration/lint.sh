#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/..")

pythonFiles=$(find "$ROOT_DIR" -name '*.py')
bashFiles=$(find "$ROOT_DIR" -name '*.sh')
cppFiles=$(find "$ROOT_DIR" -name '*.cpp' -or -name '*.hpp')

echo Running black on:
echo "$pythonFiles"
black --check --diff $pythonFiles || exit $?

echo Running shellcheck on:
echo "$bashFiles"
shellcheck -x -e SC1090,SC2086,SC2046 $bashFiles || exit $?

echo Running clang-format on:
echo "$cppFiles"
clangFormatIssues=0
for file in $cppFiles; do
    diff_output=$(diff -u "$file" <(clang-format "$file"))
    if [ -n "$diff_output" ]; then
        echo "$diff_output"
        clangFormatIssues=1
    fi
done

if [ $clangFormatIssues -ne 0 ]; then
    exit $clangFormatIssues
fi

echo Success! ["$(basename $0)"]
