#!/bin/bash

if [ -z "$CI_TEST_OS" ]; then
  echo "Env var CI_TEST_OS not supplied!"
  exit 1
fi

echo "Starting $(basename $0) with CI_TEST_OS=$CI_TEST_OS"

docker run \
    --volume $PWD:/host  \
    --workdir /host/continuous-integration  \
    $CI_TEST_OS  \
    bash -c "./code_analysis.sh" || exit $?
