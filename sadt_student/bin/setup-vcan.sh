#!/usr/bin/env sh
sudo modprobe vcan
echo "Started kernel module"
if ! ip link | grep -q vcan0; then
  sudo ip link add dev vcan0 type vcan # Add virtual CAN socket called vcan0
  sudo ip link set up vcan0 # Set vcan0 up
  echo "Created vcan0"
else
  echo "vcan0 already exist"
fi
