#! /bin/sh

ARDUINO_HOME=$HOME/sdk/arduino
ARDUINO_PORT=/dev/ttyACM0
ARDUINO_BOARD=uno
ARDUINO_BAUDRATE=9600

if [ "$1" == "console" ]; then
  miniterm.py $ARDUINO_PORT $ARDUINO_BAUDRATE
else
  scons ARDUINO_HOME=$ARDUINO_HOME ARDUINO_PORT=$ARDUINO_PORT ARDUINO_BOARD=$ARDUINO_BOARD $*
fi
