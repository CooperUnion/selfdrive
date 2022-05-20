#!/bin/sh

CANDUMPS_DIR='candumps'

args=$@
can_dev=''


# extract CAN device from passed command
while [ -n "$1" ]; do
    case "$1" in
    -c|--can)
        shift
        can_dev="$1"
        break
        ;;
    esac

    shift
done


candump -ta "$can_dev" > "$CANDUMPS_DIR/$(date +'%Y-%m-%dT%H:%M:%S%:z')" &
CANDUMP_PID="$!"

$args

kill -9 "$CANDUMP_PID"
