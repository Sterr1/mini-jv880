#
# Makefile
# export PATH=$(readlink -f ./gcc-*aarch64-none*/bin/):$PATH
# RPI=4 bash -ex build.sh
#

CIRCLE_STDLIB_DIR = ../circle-stdlib
CMSIS_DIR = ../CMSIS_5/CMSIS

OBJS = main.o kernel.o minijv880.o config.o userinterface.o uibuttons.o \
       mididevice.o midipin.o serialmididevice.o \
       emulator/lcd.o emulator/mcu.o emulator/mcu_opcodes.o emulator/pcm.o 

OPTIMIZE = -O3

include ./Rules.mk
