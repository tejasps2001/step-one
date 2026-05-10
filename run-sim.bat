@echo off
setlocal

set "CONFIG_PATH="
set "GUI=false"
set "CLEAN=false"
set "BATCH_RUNS=1"

:parse_args
if "%~1"=="" goto end_parse
if /I "%~1"=="--clean" (
    set "CLEAN=true"
    shift
    goto parse_args
)
if /I "%~1"=="-g" (
    set "GUI=true"
    shift
    goto parse_args
)
if /I "%~1"=="--gui" (
    set "GUI=true"
    shift
    goto parse_args
)
if /I "%~1"=="-b" (
    set "BATCH_RUNS=%~2"
    shift
    shift
    goto parse_args
)
if /I "%~1"=="--batch" (
    set "BATCH_RUNS=%~2"
    shift
    shift
    goto parse_args
)
if /I "%~1"=="-c" (
    set "CONFIG_PATH=%~2"
    shift
    shift
    goto parse_args
)
if /I "%~1"=="--config" (
    set "CONFIG_PATH=%~2"
    shift
    shift
    goto parse_args
)

if "%CONFIG_PATH%"=="" (
    set "CONFIG_PATH=%~1"
    shift
    goto parse_args
)

echo Unknown parameter passed: %~1
exit /b 1

:end_parse

if "%CONFIG_PATH%"=="" (
    echo [ERROR] Please provide the path to your settings config file.
    echo Usage: run-sim.bat [path\to\config.txt] [--gui] [--clean] [-b runs]
    exit /b 1
)

if "%CLEAN%"=="true" (
    echo ^>^>^> Cleaning old build directories...
    call .\gradlew clean
    echo ^>^>^> Stopping lingering Gradle daemons...
    call .\gradlew --stop
)

echo ^>^>^> Compiling and packaging standalone distribution...
call .\gradlew :step-one-main:installDist -x test

if %ERRORLEVEL% neq 0 (
    echo [ERROR] Build failed! Aborting run.
    exit /b %ERRORLEVEL%
)

echo ^>^>^> Launching Simulator...
pushd step-one-main
if "%GUI%"=="true" (
    call .\build\install\step-one-main\bin\step-one-main.bat "%CONFIG_PATH%"
) else (
    call .\build\install\step-one-main\bin\step-one-main.bat -b %BATCH_RUNS% "%CONFIG_PATH%"
)
popd