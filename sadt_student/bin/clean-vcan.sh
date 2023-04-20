#!/usr/bin/env sh

clean_vcan() {
  vcan_num="$1"

  if ip link | grep -q "vcan$vcan_num"; then
    sudo ip link set down "vcan$vcan_num"
    sudo ip link delete dev "vcan$vcan_num"
    echo "vcan$vcan_num deleted"
  else
    echo "vcan$vcan_num does not exist"
  fi
}

clean_vcan 0
clean_vcan 1
clean_vcan 2