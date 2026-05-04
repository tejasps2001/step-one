#!/bin/bash

CONFIG_PATH="samples/milp/random_shutdown.txt"
GUI=false
CLEAN=false

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--config) CONFIG_PATH="$2"; shift ;;
        -g|--gui) GUI=true ;;
        --clean) CLEAN=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Convert any backslashes to forward slashes for Linux compatibility
CONFIG_PATH="${CONFIG_PATH//\\//}"

if [ "$CLEAN" = true ]; then
    echo -e "\e[33m>>> Cleaning old build directories...\e[0m"
    ./gradlew clean
    echo -e "\e[33m>>> Stopping lingering Gradle daemons...\e[0m"
    ./gradlew --stop
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
    ./build/install/step-one-main/bin/step-one-main -b 1 "$CONFIG_PATH"
fi

popd > /dev/null
