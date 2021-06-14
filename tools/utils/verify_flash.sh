#!/bin/sh

#
# This uses openocd to dump the flash and compares it to the firmware file to
# make sure what we wanted to flash got there.
#
# $ ./verify_flash.sh [path to bin file]
#

binary="$1"
dump=$(mktemp)

croak() {
    echo "$@"
    rm "$dump"
    exit 1
}

size=$(stat --printf="%s" "$binary") || {
    croak "failed to stat file"
}

cmd="flash read_bank 0 $dump"
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c "$cmd" -c exit || {
    croak "failed to read out flash with openocd"
}

cmp -l --ignore-initial=0x4000:0 --bytes="$size" "$dump" "$binary" || {
    croak "files did not match!"
}

rm "$dump"

exit 0
