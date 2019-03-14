#!/bin/bash

export ODOMETRY_DEV=

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    syspath="${sysdevpath%/dev}"
    devname="$(udevadm info -q name -p $syspath)"
    [[ "$devname" == "bus/"* ]] && continue
    eval "$(udevadm info -q property --export -p $syspath)"
    [[ -z "$ID_VENDOR_ID" ]] && continue
    echo "/dev/$devname - $ID_VENDOR_ID:$ID_MODEL_ID"
    if [[ "$ID_VENDOR_ID:$ID_MODEL_ID" == "067b:2303" ]]; then
        export ODOMETRY_DEV=/dev/$devname
        echo "  Found odometry dev : $ODOMETRY_DEV"
    fi
done
