#!/bin/bash

echo Start ["$(basename $0)"]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )" || exit $?

cd ~/ros2_ws || exit $?

echo "Installing Zivid API config file"
install -D "$SCRIPT_DIR"/ZividAPIConfigCPU.yml "$HOME"/.config/Zivid/API/Config.yml || exit $?

echo "Download and install zivid sample data"
for sample in "FileCameraZivid2M70.zip" "FileCameraZivid2M70_HDR.zip" "BinWithCalibrationBoard.zip"; do
    echo "Downloading ${sample}"
    wget -q "https://www.zivid.com/software/${sample}" || exit $?
    mkdir -p /usr/share/Zivid/data/ || exit $?
    unzip "./${sample}" -d /usr/share/Zivid/data/ || exit $?
    rm "./${sample}" || exit $?
done

echo "Running tests"

# We exclude `clang_format` here since it has variations between versions, instead we check it during code analysis.
excludeTests="clang_format"

if [[ "$CI_TEST_COMPILER" == "g++"* ]]; then
  # When using gcc, ament_clang_tidy reports errors with missing system headers. Hence, restrict to clang builds only.
  echo "Skipping clang-tidy tests since compiler is '$CI_TEST_COMPILER'"
  excludeTests+="|clang_tidy"
fi

export GTEST_BREAK_ON_FAILURE=1;
colcon test --event-handlers console_direct+ --ctest-args tests --exclude-regex $excludeTests --output-on-failure || exit $?

echo "Check for test errors"
colcon test-result --all || exit $?

echo Success! ["$(basename $0)"]
