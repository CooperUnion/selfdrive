#!/bin/sh

CANDUMPS_DIR='candumps'
CANDUMP_PADDING='1'

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

echo "ADDING PADDING OF '$CANDUMP_PADDING' SECOND(S) TO CANDUMP"
sleep "$CANDUMP_PADDING"

echo "STARTING TEST: '$args'"
$args
echo "TEST COMPLETE"

echo "ADDING PADDING OF '$CANDUMP_PADDING' SECOND(S) TO CANDUMP"
sleep "$CANDUMP_PADDING"

kill -9 "$CANDUMP_PID"
