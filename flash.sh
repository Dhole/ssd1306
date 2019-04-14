#!/bin/sh

set -ex

NAME=`basename ${PWD}`
NAME="$1"

cargo build --example ${NAME} --release
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/examples/${NAME} ${NAME}.bin

# stlink version
# st-flash erase
# st-flash write ${NAME}.bin 0x8000000

# OpenOCD version
# http://openocd.org/doc/html/Flash-Programming.html
openocd -f openocd.cfg -c "program ${NAME}.bin reset exit 0x8000000"
