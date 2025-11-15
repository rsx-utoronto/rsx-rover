#!/bin/bash
# Linux/Mac shell script to run the A* simulator

echo "========================================"
echo "  A* Obstacle Avoidance Simulator"
echo "========================================"
echo ""

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 is not installed"
    echo "Please install Python 3.7 or higher"
    exit 1
fi

echo "Python found: $(python3 --version)"
echo ""

# Check if dependencies are installed
echo "Checking dependencies..."
if ! python3 -c "import numpy, matplotlib, PyQt5" &> /dev/null; then
    echo ""
    echo "Dependencies not found. Installing..."
    echo ""
    python3 -m pip install -r simulator_requirements.txt
    if [ $? -ne 0 ]; then
        echo ""
        echo "ERROR: Failed to install dependencies"
        echo "Please run manually: pip3 install numpy matplotlib PyQt5"
        exit 1
    fi
fi

echo "Dependencies OK!"
echo ""
echo "Launching simulator..."
echo ""

# Run the simulator
python3 astar_simulator.py

if [ $? -ne 0 ]; then
    echo ""
    echo "Simulator exited with error"
    read -p "Press Enter to continue..."
fi

