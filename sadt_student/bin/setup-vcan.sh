#!/usr/bin/env sh
sudo modprobe vcan
echo "Started kernel module"

create_vcan() {
  vcan_num="$1"
  vcan_mtu="$2"

  if ! ip link | grep -q "vcan$vcan_num"; then
    sudo ip link add dev "vcan$vcan_num" type vcan # Add virtual CAN socket 
    if [ -z "$vcan_mtu" ]; then
      sudo ip link set up "vcan$vcan_num" # Set vcan up
      echo "Created vcan$vcan_num"
    else
      sudo ip link set up "vcan$vcan_num" mtu "$vcan_mtu" # Set vcan up with custom mtu
      echo "Created vcan$vcan_num with mtu $vcan_mtu"
    fi
  else
    echo "vcan$vcan_num already exist"
  fi
}

create_vcan 0
create_vcan 1
# create_vcan 2 72