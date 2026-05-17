#!/bin/bash

CONFIG_PATH=""
GUI=false
CLEAN=false
BATCH_RUNS="1"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--config) CONFIG_PATH="$2"; shift ;;
        -g|--gui) GUI=true ;;
        --clean) CLEAN=true ;;
        -b|--batch) BATCH_RUNS="$2"; shift ;;
        -*) echo "Unknown parameter passed: $1"; exit 1 ;;
        *) CONFIG_PATH="$1" ;;
    esac
    shift
done

# Convert any backslashes to forward slashes for Linux compatibility
CONFIG_PATH="${CONFIG_PATH//\\//}"

if [ "$CLEAN" = true ]; then
    echo -e "\e[33m>>> Stopping lingering Gradle daemons...\e[0m"
    ./gradlew --stop
    echo -e "\e[33m>>> Cleaning old build directories...\e[0m"
    rm -rf build step-one-main/build the-one/build sim-flowable/build
    if [ -z "$CONFIG_PATH" ]; then
        exit 0
    fi
fi

if [ -z "$CONFIG_PATH" ]; then
    echo -e "\e[31m[ERROR] Please provide the path to your settings config file.\e[0m"
    echo "Usage: ./run-sim.sh [path/to/config.txt] [--gui] [--clean] [-b runs]"
    exit 1
fi

echo -e "\e[36m>>> Compiling and packaging standalone distribution...\e[0m"
./gradlew :step-one-main:installDist -x test

if [ $? -ne 0 ]; then
    echo -e "\e[31mBuild failed! Aborting run.\e[0m"
    exit 1
fi

echo -e "\e[36m>>> Launching Simulator...\e[0m"
pushd step-one-main > /dev/null || exit

# Note: On Linux, the executable doesn't have the .bat extension
if [ "$GUI" = true ]; then
    ./build/install/step-one-main/bin/step-one-main "$CONFIG_PATH"
else
    ./build/install/step-one-main/bin/step-one-main -b "$BATCH_RUNS" "$CONFIG_PATH"
fi

popd > /dev/null
