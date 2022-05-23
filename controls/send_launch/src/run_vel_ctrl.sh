#!/bin/sh
CANDUMPS_DIR='candumps'

args=$@
can_dev=''
kp=''
ki=''
kd=''
vd=''


# extract CAN device from passed command
while [ -n "$1" ]; do
    case "$1" in
    --kp)
        shift
        kp="$1"
        ;;
    --ki)
        shift
        ki="$1"
        ;;
    --kd)
        shift
        kd="$1"
        ;;
    --vd)
        shift
        vd="$1"
        ;;
    esac

    shift
done


candump -ta "can0" > "$CANDUMPS_DIR/kp_${kp}_ki_${ki}_kd_${kd}_vd_${vd}" &
CANDUMP_PID="$!"

sleep 2
$args
sleep 2

kill -9 "$CANDUMP_PID"
