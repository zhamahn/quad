#! /bin/sh

DIR=$(dirname $0)

ARDUINO_HOME=$HOME/sdk/arduino
ARDUINO_PORT=/dev/ttyACM0
ARDUINO_BOARD=uno
EXTRA_LIB_DIR=$DIR/lib

scons ARDUINO_HOME=$ARDUINO_HOME ARDUINO_PORT=$ARDUINO_PORT ARDUINO_BOARD=$ARDUINO_BOARD EXTRA_LIB=$EXTRA_LIB_DIR $*
