#!/bin/bash
echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?

$SCRIPT_DIR/setup/setup_build_and_test.sh || exit $?

$SCRIPT_DIR/build.sh || exit $?

$SCRIPT_DIR/test.sh || exit $?

echo Success! ["$(basename $0)"]