#!/bin/bash
PATH=/home/nlbutts/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin:$PATH
cd ../build
arm-none-eabi-gdb -tui --command=../scripts/gdbinit software.elf

