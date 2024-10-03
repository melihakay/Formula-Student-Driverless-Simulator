#!/bin/bash

# Check if the first argument is provided; if not, assign a default value
export MFR_FSDS_MAP=${1:-"acceleration"}
export MFR_FSDS_MODE=${2:-"graphical"}

echo "Running WheelieSim"
echo "Map: $MFR_FSDS_MAP" 
echo "Mode: $MFR_FSDS_MODE"

# Run the zellij command with the specified layout
cd ~/Formula-Student-Driverless-MFR/run
zellij -l fsds.kdl
