/*
  RadioProxy - Arduino libary for mapping 433Mhz devices and Mozilla
  WebThings. It uses RCSwitch for communication through the
  oddWires IoT-Bus 433Mhz board based on the Hope RFM69 module.
  Copyright (c) 2018 ian Archbell, oddWires.  All right reserved.

  Original RCSwitch project home: https://github.com/sui77/rc-switch/
  Also see copyright and license notices in RCSwitch sources.

  Project home for iot-bus: https://github.com/iot-bus/rc-switch/

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

#include <WebThingAdapter.h>
#include <RCSwitch.h>

// Radio transmit and receive pin (RFM69 DIO2)
#define RADIOPROXY_RADIOPIN 4

enum ProxyType { PROXY_INPUT, PROXY_OUTPUT };

class RadioProxy{
 
   public:
    RadioProxy(ProxyType proxyType, ThingProperty* property, uint32_t onCode, uint32_t offCode, 
              int pulseLength = 186, int protocol = 1, int codeLength = 24, int repetitions = 10);
    ~RadioProxy();
    ProxyType proxyType();
    ThingProperty* property();
    uint32_t offCode();
    uint32_t onCode();
    bool isOnCode(uint32_t code);
    bool isOffCode(uint32_t code);

    int pulseLength();
    int protocol();
    int codeLength();
    int repetitions();
    void setPulseLength(int pulseLength);
    void setProtocol(int protocol);
    void setCodeLength(int codeLength);
    void setRepetitions(int repetitions);
    bool status();
  
    static void mapPropertyStatus();
    static void mapPropertyStatus(ThingProperty* property);
    static int mapRadioStatus();
    static void enableRadio(int pin = RADIOPROXY_RADIOPIN);

    static void sendCodeToProxy(RadioProxy* proxy, uint32_t code);
  
  private:
    ProxyType _proxyType;
    ThingProperty* _property;
    uint32_t _offCode;
    uint32_t _onCode;
    int _pulseLength;
    int _codeLength;
    int _protocol;
    int _repetitions;
    bool _status;
    static int proxyCount; 

    static RCSwitch theRadio;
    static bool radioEnabled;

    void setStatus(bool status);
    void removeProxy(RadioProxy* proxy);
    static RadioProxy* getProxyForCode(uint32_t code);
    static RadioProxy* getProxyForProperty(ThingProperty* property);
    static std::vector<RadioProxy*>* getProxies();
 
};