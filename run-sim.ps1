param (
    [Parameter(Mandatory=$false)]
    [string]$ConfigPath = "samples\milp\random_shutdown.txt",
    
    [Parameter(Mandatory=$false)]
    [switch]$GUI,

    [Parameter(Mandatory=$false)]
    [switch]$Clean
)

if ($Clean) {
    Write-Host ">>> Cleaning old build directories..." -ForegroundColor Yellow
    .\gradlew clean
    Write-Host ">>> Stopping lingering Gradle daemons..." -ForegroundColor Yellow
    .\gradlew --stop
}

Write-Host ">>> Compiling and packaging standalone distribution..." -ForegroundColor Cyan
.\gradlew :step-one-main:installDist -x test

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed! Aborting run." -ForegroundColor Red
    exit $LASTEXITCODE
}

Write-Host ">>> Launching Simulator..." -ForegroundColor Cyan
Push-Location step-one-main
if ($GUI) {
    .\build\install\step-one-main\bin\step-one-main.bat $ConfigPath
} else {
    .\build\install\step-one-main\bin\step-one-main.bat -b 1 $ConfigPath
}
Pop-Location