#!/bin/bash

export ODOMETRY_DEV=
export RPLIDAR_DEV=
export STLINK_DEV=

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    syspath="${sysdevpath%/dev}"
    devname="$(udevadm info -q name -p $syspath)"
    [[ "$devname" == "bus/"* ]] && continue
    [[ "$devname" != "tty"* ]] && continue
    eval "$(udevadm info -q property --export -p $syspath)"
    [[ -z "$ID_VENDOR_ID" ]] && continue
    echo "/dev/$devname - $ID_VENDOR_ID:$ID_MODEL_ID"
    if [[ "$ID_VENDOR_ID:$ID_MODEL_ID" == "0403:6001" ]]; then
        export ODOMETRY_DEV=/dev/$devname
        echo "  Found odometry dev : $ODOMETRY_DEV"
    fi
    if [[ "$ID_VENDOR_ID:$ID_MODEL_ID" == "10c4:ea60" ]]; then
        export RPLIDAR_DEV=/dev/$devname
        echo "  Found rplidar dev : $RPLIDAR_DEV"
    fi
    if [[ "$ID_VENDOR_ID:$ID_MODEL_ID" == "0483:374b" ]]; then
        export STLINK_DEV=/dev/$devname
        echo "  Found stlink dev : $STLINK_DEV"
    fi
done
