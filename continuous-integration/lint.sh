#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/..")

pythonFiles=$(find "$ROOT_DIR" -name '*.py')
bashFiles=$(find "$ROOT_DIR" -name '*.sh')

echo Running black on:
echo "$pythonFiles"
black --check --diff $pythonFiles || exit $?

echo Running shellcheck on:
echo "$bashFiles"
shellcheck -x -e SC1090,SC2086,SC2046 $bashFiles || exit $?

echo Running code analysis on C++ code:
$SCRIPT_DIR/lint_cpp.sh || exit $?

echo Success! ["$(basename $0)"]
