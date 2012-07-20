#! /bin/sh
avr-g++ -o build/main.o -c -ffunction-sections -fdata-sections -fno-exceptions -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Os -mmcu=atmega328p -DARDUINO=100 -DF_CPU=16000000L -Ibuild/core -I/home/zhamahn/sdk/arduino/hardware/arduino/variants/standard -Ibuild/lib_00/Wire -Ibuild/lib_00/Wire/utility -Ibuild/lib_01/PID_v1 main.cpp
