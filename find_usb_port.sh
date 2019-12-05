#!/bin/bash

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        # echo "$ID_SERIAL"
        if [ $ID_SERIAL == "FTDI_FT230X_Basic_UART_DO01F2RQ" ]; then
            echo "$devname"
        fi
    )
done

