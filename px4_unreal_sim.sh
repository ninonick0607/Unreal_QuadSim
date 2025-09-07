#!/bin/bash

# PX4 Unreal Sim Launch Script
# This script starts PX4 SITL with proper configuration for Unreal Engine simulation

echo "Starting PX4 for Unreal HIL simulation..."

# Kill any existing PX4 processes (but not this script!)
echo "Cleaning up old processes..."
pkill -f "px4_sitl|bin/px4" 2>/dev/null || true
pkill -f simulator_mavlink 2>/dev/null || true
sleep 1

# Navigate to PX4 directory
cd ~/PX4-Autopilot

# Clean previous parameters
echo "Cleaning parameters..."
rm -f parameters.bson parameters_backup.bson

# Set environment to disable timeouts in simulator_mavlink
export PX4_SIM_SPEED_FACTOR=0  # This disables lockstep timing
export MAV_SIM_NO_TIMEOUT=1     # Disable timeout checks

# Run standard PX4 SITL with none_iris model
echo "Starting PX4 SITL..."
exec make px4_sitl none_iris