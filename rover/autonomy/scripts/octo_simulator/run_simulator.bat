@echo off
REM Windows batch file to run the A* simulator
REM Double-click this file to launch the simulator

echo ========================================
echo   A* Obstacle Avoidance Simulator
echo ========================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7 or higher from python.org
    pause
    exit /b 1
)

echo Python found!
echo.

REM Check if dependencies are installed
echo Checking dependencies...
python -c "import numpy, matplotlib, PyQt5" >nul 2>&1
if errorlevel 1 (
    echo.
    echo Dependencies not found. Installing...
    echo.
    python -m pip install -r simulator_requirements.txt
    if errorlevel 1 (
        echo.
        echo ERROR: Failed to install dependencies
        echo Please run manually: pip install numpy matplotlib PyQt5
        pause
        exit /b 1
    )
)

echo Dependencies OK!
echo.
echo Launching simulator...
echo.

REM Run the simulator
python astar_simulator.py

if errorlevel 1 (
    echo.
    echo Simulator exited with error
    pause
)

