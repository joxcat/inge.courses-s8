#!/usr/bin/env sh
if ip link | grep -q vcan0; then
  sudo ip link set down vcan0
  sudo ip link delete dev vcan0
  echo "vcan0 deleted"
else
  echo "vcan0 does not exist"
fi
