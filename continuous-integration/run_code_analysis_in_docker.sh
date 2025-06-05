#!/bin/bash

if [ -z "$CI_TEST_OS" ]; then
  echo "Env var CI_TEST_OS not supplied!"
  exit 1
fi

echo "Starting $(basename $0) with CI_TEST_OS=$CI_TEST_OS"

echo "Fixing expired GPG key before running apt-get update"
mkdir -p ./continuous-integration/setup/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | gpg --dearmor -o ./continuous-integration/setup/keyrings/ros-archive-keyring.gpg

echo "Running the code analysis script in a Docker container"
docker run \
    --volume $PWD:/host  \
    --volume $PWD/continuous-integration/setup/keyrings/ros-archive-keyring.gpg:/usr/share/keyrings/ros1-latest-archive-keyring.gpg \
    --workdir /host/continuous-integration  \
    $CI_TEST_OS  \
    bash -c "./code_analysis.sh" || exit $?

echo "Cleaning up keyrings"
rm -rf ./continuous-integration/setup/keyrings
