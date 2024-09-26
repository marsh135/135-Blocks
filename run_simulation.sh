#!/bin/bash

# Run the FRC simulator in the background
./gradlew simulateJava &

# Capture the PID of the simulator
SIMULATOR_PID=$!

# Allow the simulator to run for 30 seconds
sleep 30

# Stop the simulator
kill $SIMULATOR_PID