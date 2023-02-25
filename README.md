# Tone 5 Throwbot Firmware

Pirates need to find that treasure fast before Uranus stops aligning.

# Arduino Modifications

Firmware is written in, compiled by, and uploaded using the Arduino IDE.
There are several modifications made to the stock packages.

## Older version of ArduinoBLE

Follow the bottom post in [this forum](https://forum.sparkfun.com/viewtopic.php?f=169&t=54311).

## Time queue limit

Change `OS_TIMER_CB_QUEUE` in `RTX_Config.h` to 64.
