#!/bin/bash

if [ -z "$CI_TEST_ZIVID_VERSION" ]; then
  echo "Env var CI_TEST_ZIVID_VERSION not supplied!"
  exit 1
fi

if [ -z "$CI_TEST_OS" ]; then
  echo "Env var CI_TEST_OS not supplied!"
  exit 1
fi

if [ -z "$CI_TEST_COMPILER" ]; then
  echo "Env var CI_TEST_COMPILER not supplied!"
  exit 1
fi

echo "Starting $(basename $0)"
echo "CI_TEST_ZIVID_VERSION=$CI_TEST_ZIVID_VERSION"
echo "CI_TEST_OS=$CI_TEST_OS"
echo "CI_TEST_COMPILER=$CI_TEST_COMPILER"
echo "CI_TEST_DOWNLOAD_TELICAM=$CI_TEST_DOWNLOAD_TELICAM"

docker run \
    --volume $PWD:/host  \
    --workdir /host/continuous-integration  \
    --env CI_TEST_ZIVID_VERSION="$CI_TEST_ZIVID_VERSION" \
    --env CI_TEST_COMPILER="$CI_TEST_COMPILER" \
    --env CI_TEST_DOWNLOAD_TELICAM="$CI_TEST_DOWNLOAD_TELICAM" \
    $CI_TEST_OS  \
    bash -c "./build_and_test.sh" || exit $?
