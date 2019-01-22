# Brushless-Dom-Nerf-esp32
A brushless dominator code port of https://github.com/airzone-sama/Brushless_Dominator to the ESP32. 

The Sparkfun ESP32 Thing was the only unit to test successfull



# Original credit and many thanks go to https://github.com/airzone-sama for a ton of help and advice.

## A HOW-TO And Bill Of Materials, and maybe a wiring video and doc will appear by March 2019.


---------------------------------------
From the original readme.md

# Brushless_Dominator
Firmware for Airzone's Brushless Dominator


An advanced modification for the Worker Dominator to allow for a sensored triple brushless w/ solenoid pusher, OLED, bluetooth, etc. 

Use at your own risk. This blaster is capable of seriously injuring a person if used incorrectly.

The excel file shows the wiring configuration. Please note that I use the following:

1) This is based on the STM32. In particular the Blue Pill variant (although any can be used)
2) All IR LEDs / Sensor Emitters need a 100 ohm resistor at 3.3v
3) Voltage detection uses a voltage divider with 47k and 10k resistors.
4) Bluetooth module is a HC-05
6) ESC's are blHeli - S is OK, but 32 is recommended.
7) You want a big battery for a triple build
8) You have your own choice of power supply. I use a cheap Chinese 5v/3a BEC, and run it through a 3.3v LDO


License:
Use at your own risk.

There is to be no commercial activity of any kind using this firmware, or any other derived firmware in any blaster. Please contact me to make alternate license arrangements.
