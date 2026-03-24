#!/bin/bash

# Flash a swarm of Crazyflies
# Usage: ./flash_swarm.sh [count]

COUNT=${1:-9}
BASE_ADDRESS="ABAD1DEA"
CHANNEL=90

FLASHED=()
FAILED=()

for i in $(seq 1 $COUNT); do
    ADDR=$(printf "%02d" $i)
    URI="radio://0/$CHANNEL/2M/${BASE_ADDRESS}${ADDR}"
    echo "Flashing Crazyflie $i: $URI"
    if CLOAD_CMDS="-w $URI" make cload; then
        FLASHED+=($i)
    else
        FAILED+=($i)
    fi
done

echo ""
echo "===== Flashing Summary ====="
if [ ${#FLASHED[@]} -gt 0 ]; then
    echo "Successfully flashed: ${FLASHED[*]}"
else
    echo "Successfully flashed: none"
fi

if [ ${#FAILED[@]} -gt 0 ]; then
    echo "Failed to flash:      ${FAILED[*]}"
else
    echo "Failed to flash:      none"
fi