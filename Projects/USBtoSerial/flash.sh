#!/bin/sh
avrdude -c avrispv2 -p atmega32u2 -P /dev/ttyUSB0 -U flash:w:USBtoSerial.hex -U lfuse:w:0xff:m -U hfuse:w:0xd9:m
