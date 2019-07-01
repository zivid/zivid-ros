#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?

$SCRIPT_DIR/setup/setup_code_analysis.sh || exit $?

$SCRIPT_DIR/lint.sh || exit $?

echo Success! ["$(basename $0)"]
