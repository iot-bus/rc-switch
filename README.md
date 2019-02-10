# rc-switch



This documentation is for https://github/iot-bus/rc-switch. This version adds the capability of RadioProxy to map Mozilla 
WebThings to 433Mhz devices, essentially providing a gateway to and from 433Mhz devices. 

RadioProxy uses a modified version of the RCSwitch library available here: https://github/sui77/rc-switch. 
This version of RCSwitch has been modified for use with Hope RFM69 modules which contain a transceiver in small format. 
This version of RCSwitch incorporates mush of Kobuki's work on OOK which can be found here: https://github.com/kobuki/RFM69OOK.

The cheap devices widely available at 433Mhz typically use a code for on and a code for off. These codes are in mark/space format where 
a bit is represented by a combination of high/low spaces and vary in terms of pulse duration. RCSwitch handles the low level sending 
and receipt of these codes as well as decoding the protocol used.

RadioProxy is a new class added to RCSwitch which statically maps 433Mhz codes to Mozilla WebThings properties both for sending and receiving. 

Let's see hw to define a 433Mhz device for use with WebThings. Here is a simple WebThing controller with two properties:

    ThingDevice lightController("LightController", "IoT-Bus 433Mhz Light Controller", sensorTypes);
    ThingProperty control1("Control 1", "433Mhz Light Control 1", BOOLEAN, "BooleanProperty");
    ThingProperty control2("Control 2", "433Mhz Light Control 2", BOOLEAN, "BooleanProperty");

This is how you add the proxies or the devices that map 433Mhz codes to properties and vice versa:

    RadioProxy proxy5(PROXY_INPUT, &control1, 1135923, 1135932);
    RadioProxy proxy6(PROXY_INPUT, &control2, 1136067, 1136076);

These declarations indicate that radio input of these on codes and off codes are to be mapped to control1 
and control2 property respectively.

Note that is=t is possible to map "flip-flop" type devices that have a single code rather than separate on and off codes:

    RadioProxy proxy1(PROXY_INPUT, &buttonProperty1, 15864961);

RadioProxy flips the state each time it gets this type of code.  

Output devices are declared in a similar way but require more information. This is a power outlet with typical WebThings declaration.

    ThingDevice outlet1("Outlet 1", "IoT-Bus 433Mhz Outlet 1", sensorTypes);
    ThingProperty power1("Power", "433Mhz Outlet 1", BOOLEAN, "BooleanProperty");

Here is the proxy for that WebThing. PROXY_OUTPUT indicates it is an output device, then there's the reference to the WebThing being mapped and the on and off codes.
After that comes the bit length, typically 24, the pulse length which varies a lot, then the protocol number. This is the protocol number that RCSwitch uses.
You can find the protocol table in protocol.h which is part of RCSwitch. Most devices are protocol 1. The last item is the number of repetitions that should be used.
Ten is normally fine. 

So how do you find out what the on/off codes are? Run any of these examples and press the control button.
If the code is detected, it will show on the serial monitor. You can then use the codes in your proxy declarations. 
So it's quite straightforward to declare the RadioProxies. 

    RadioProxy proxy16(PROXY_OUTPUT, &power1, 5264691, 5264700, 24, 184, 1, 10);

To use the 433Mhz radio, you need to turn it on in your setup()

    RadioProxy::begin(true); // start radio in verbose more

The mapping or gateway code is handled by this statement in your loop()

    RadioProxy::update();

And that's it!

This fork is only designed to run on ESP32, specifically the IoT-Bus 433Mhz board. Use the original RCSwitch library if you need 
other platform support.

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
