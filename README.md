# rc-switch

This fork of rc-switch enables the Hope RFM69 modules to be used to control and sense the cheap 433Mhz modules widely available.
It incorporate a modified version of Kobuki's RFM96OOK OOK library which itself is a version of LowPowerLabs. Kudos to the developers involved.
This version has also been created to make it very simple to use the new IoT-Bus 433Mhz/915MHz boards with OOK and decoding/encoding many of the most common device protocols courtesy of the original rc-switch.

The original rc-switch library information is below:

[![Build Status](https://travis-ci.org/sui77/rc-switch.svg?branch=master)](https://travis-ci.org/sui77/rc-switch)

Use your Arduino or Raspberry Pi to operate remote radio controlled devices

## Download
https://github.com/sui77/rc-switch/releases/latest

rc-switch is also listed in the arduino library manager.

## Wiki
https://github.com/sui77/rc-switch/wiki

## Info
### Send RC codes

Use your Arduino or Raspberry Pi to operate remote radio controlled devices.
This will most likely work with all popular low cost power outlet sockets. If
yours doesn't work, you might need to adjust the pulse length.

All you need is a Arduino or Raspberry Pi, a 315/433MHz AM transmitter and one
or more devices with one of the supported chipsets:

 - SC5262 / SC5272
 - HX2262 / HX2272
 - PT2262 / PT2272
 - EV1527 / RT1527 / FP1527 / HS1527 
 - Intertechno outlets
 - HT6P20X

### Receive and decode RC codes

Find out what codes your remote is sending. Use your remote to control your
Arduino.

All you need is an Arduino, a 315/433MHz AM receiver (altough there is no
instruction yet, yes it is possible to hack an existing device) and a remote
hand set.

For the Raspberry Pi, clone the https://github.com/ninjablocks/433Utils project to
compile a sniffer tool and transmission commands.

Kobuki's RFM96OOK library is available here:

https://github.com/kobuki/RFM69OOK
