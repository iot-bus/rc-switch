/*
  This fork of RCSwitch is designed to run on IoT-Bus. Default pins are set for
  oddWires IoT-Bus 433Mhz board using a Hope RFM69 module. 
  
  The OOK capability is based on the work done by Kobuki here: 
  https://github.com/kobuki/RFM69OOK which itself is derived for the work done by LowPowerLabs
  here: https://github.com/LowPowerLab/RFM69

  RadioProxy can be used with this version of RCSwitch to enable mapping between 
  Mozilla WebThings and 433Mhz controllers and devices. 
  RadioProxy is part of this fork of rc-switch.
  
  RCSwitch - Arduino libary for remote control outlet switches
  Copyright (c) 2011 Suat Özgür.  All right reserved.

  Contributors:
  - Andre Koehler / info(at)tomate-online(dot)de
  - Gordeev Andrey Vladimirovich / gordeev(at)openpyro(dot)com
  - Skineffect / http://forum.ardumote.com/viewtopic.php?f=2&t=46
  - Dominik Fischer / dom_fischer(at)web(dot)de
  - Frank Oltmanns / <first name>.<last name>(at)gmail(dot)com
  - Max Horn / max(at)quendi(dot)de
  - Robert ter Vehn / <first name>.<last name>(at)gmail(dot)com
  
  Original project home: https://github.com/sui77/rc-switch/

  Project home for RCSwitch for iot-bus: https://github.com/iot-bus/rc-switch/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _RCSwitch_h
#define _RCSwitch_h

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <stdint.h>

// Number of maximum high/Low changes per packet.
// We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
// Number of maximum high/Low changes per packet.
// We can handle up to (unsigned long) => 40 bit * 2 H/L changes per bit + 2 for sync
#define RCSWITCH_MAX_CHANGES 82

// Pulse width tolerance
#define RCSWITCH_RECEIVE_TOLERANCE 60

// Gap between packets
#define RCSWITCH_SEPARATION_LIMIT 4300

// Ignore glitches - ignore pulses less than this length in microseconds
#define RCSWITCH_MIN_DURATION 145

// IoT-Bus pins used
#define RCSWITCH_RESETPIN 13 // Was 17
#define RCSWITCH_DATAPIN 25  // Was 4
#define RCSWITCH_SS 12       // Was 5
class RCSwitch {

  public:
    //RCSwitch();
    RCSwitch(int ssPin = RCSWITCH_SS, int resetPin = RCSWITCH_RESETPIN);
    
    // RFM69

    static volatile int RSSI; //most accurate RSSI during reception (closest to the reception)
    static RCSwitch* selfPointer;

    bool initialize(); 
    void setFixedThreshold(uint8_t threshold);
    void setFrequencyMHz(float f);
    void setFrequency(uint32_t freqHz);
    void setBitrate(uint32_t bitrate);
    void setBandwidth(uint8_t bw);
    void setRSSIThreshold(int8_t rssi);
    void receiveEnd();
    void setMode(byte newMode);
    byte readReg(byte addr);
    void writeReg(byte addr, byte value);
    void select();
    void unselect();
    void setHighPower(bool onOff);
    void setHighPowerRegs(bool onOff);
    int8_t readRSSI(bool forceTrigger); 

    // RCSwitch

    void switchOn(int nGroupNumber, int nSwitchNumber);
    void switchOff(int nGroupNumber, int nSwitchNumber);
    void switchOn(const char* sGroup, int nSwitchNumber);
    void switchOff(const char* sGroup, int nSwitchNumber);
    void switchOn(char sFamily, int nGroup, int nDevice);
    void switchOff(char sFamily, int nGroup, int nDevice);
    void switchOn(const char* sGroup, const char* sDevice);
    void switchOff(const char* sGroup, const char* sDevice);
    void switchOn(char sGroup, int nDevice);
    void switchOff(char sGroup, int nDevice);

    void sendTriState(const char* sCodeWord);
    void send(unsigned long code, unsigned int length);
    void send(const char* sCodeWord);

    void enableRadio(int interruptPin = RCSWITCH_DATAPIN);
    void enableReceive(int interruptPin);
    void enableReceive();
    void disableReceive();
    
    bool available();
    void resetAvailable();

    unsigned long getReceivedValue();
    unsigned int getReceivedBitlength();
    unsigned int getReceivedDelay();
    unsigned int getReceivedProtocol();
    unsigned int* getReceivedRawdata();
  
    void setPulseLength(int nPulseLength);
    void setRepeatTransmit(int nRepeatTransmit);
    void setReceiveTolerance(int nPercent);

    /**
     * Description of a single pule, which consists of a high signal
     * whose duration is "high" times the base pulse length, followed
     * by a low signal lasting "low" times the base pulse length.
     * Thus, the pulse overall lasts (high+low)*pulseLength
     */
    struct HighLow {
        uint8_t high;
        uint8_t low;
    };

    /**
     * A "protocol" describes how zero and one bits are encoded into high/low
     * pulses.
     */
    struct Protocol {
        /** base pulse length in microseconds, e.g. 350 */
        uint16_t pulseLength;

        HighLow syncFactor;
        HighLow zero;
        HighLow one;

        /**
         * If true, interchange high and low logic levels in all transmissions.
         *
         * By default, RCSwitch assumes that any signals it sends or receives
         * can be broken down into pulses which start with a high signal level,
         * followed by a a low signal level. This is e.g. the case for the
         * popular PT 2260 encoder chip, and thus many switches out there.
         *
         * But some devices do it the other way around, and start with a low
         * signal level, followed by a high signal level, e.g. the HT6P20B. To
         * accommodate this, one can set invertedSignal to true, which causes
         * RCSwitch to change how it interprets any HighLow struct FOO: It will
         * then assume transmissions start with a low signal lasting
         * FOO.high*pulseLength microseconds, followed by a high signal lasting
         * FOO.low*pulseLength microseconds.
         */
        bool invertedSignal;
    };

    void setProtocol(Protocol protocol);
    void setProtocol(int nProtocol);
    void setProtocol(int nProtocol, int nPulseLength);

  protected: // RFM69
    static volatile byte _mode; //should be protected?

    byte _slaveSelectPin;
    byte _powerLevel;
    bool _isRFM69HW;
    byte _SPCR;
    byte _SPSR; 

  private:
    char* getCodeWordA(const char* sGroup, const char* sDevice, bool bStatus);
    char* getCodeWordB(int nGroupNumber, int nSwitchNumber, bool bStatus);
    char* getCodeWordC(char sFamily, int nGroup, int nDevice, bool bStatus);
    char* getCodeWordD(char group, int nDevice, bool bStatus);
    void transmit(HighLow pulses);

    #if not defined( RCSwitchDisableReceiving )
    static void handleInterrupt();
    void reset(void);
    static bool receiveProtocol(const int p, unsigned int changeCount);
    int nReceiverInterruptPin;
    #endif
    int nTransmitterPin;
    int nRepeatTransmit;
    int nResetPin;
    
    Protocol _protocol;

    #if not defined( RCSwitchDisableReceiving )
    static int nReceiveTolerance;
    volatile static unsigned long nReceivedValue;
    volatile static unsigned int nReceivedBitlength;
    volatile static unsigned int nReceivedDelay;
    volatile static unsigned int nReceivedProtocol;
    const static unsigned int nSeparationLimit;
    /* 
     * timings[0] contains sync timing, followed by a number of bits
     */
    static uint32_t timings[RCSWITCH_MAX_CHANGES];
    #endif
    
};

#endif
