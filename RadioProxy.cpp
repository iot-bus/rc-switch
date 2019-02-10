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
#include "RadioProxy.h"

int RadioProxy::proxyCount = 0;

RCSwitch RadioProxy::theRadio(RADIOPROXY_RESETPIN);
bool RadioProxy::radioEnabled = 0;
bool RadioProxy::_verbose = true;


/**
  * Constructor
  * PROXY_INPUT or PROXY_OUTPUT, WebThing Property, on-code and off-code
  * Other parameters required by 433Mhz devices include pulseLength and number of code repetitions.
  * These are set on enableRadio.
  * An instance list of all instances of RadioProxy is mainted for a proxy lookup table.
  */
    RadioProxy::RadioProxy(ProxyType proxyType, ThingProperty* property, uint32_t onCode, uint32_t offCode, 
                              int codeLength, int pulseLength, int protocol, int repetitions){
    _proxyType = proxyType;
    _property = property;
    _onCode = onCode;
    _offCode = offCode;
    _pulseLength = pulseLength;
    _codeLength = codeLength;
    _protocol = protocol;
    _repetitions = repetitions;
    _state = false;
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    proxies->push_back(this);
    proxyCount++;
}

/**
  * Destructor
  * Remove proxy from the instance list
  */
RadioProxy::~RadioProxy(){
    removeProxy(this);
}

/**
  * Enable the radio through RCSwitch
  */
void RadioProxy::enableRadio(int radioPin, bool verbose){
    _verbose = verbose;
    if(!radioEnabled){
        // enable the radio
        theRadio.enableRadio(radioPin);
        radioEnabled = true;
    }
}

/**
  * Return the WebThing property for this proxy
  */
ThingProperty* RadioProxy::property(){
    return _property;
}

/**
  * Return the off code for this proxy
  */
uint32_t RadioProxy::offCode(){
    return _offCode;
}

/**
  * Return the on code for this proxy
  */
uint32_t RadioProxy::onCode(){
    return _onCode;
}

/**
  * Returns true if code is an on code
  */
bool RadioProxy::isOnCode(uint32_t code){
    return (code == _onCode);
}

/**
  * Returns true if code is a flip-flop
  */
bool RadioProxy::isFlipFlopCode(uint32_t code){
    return (code != 0 && _offCode == 0);
}

/**
  * Returns true if code is an off code
  */
bool RadioProxy::isOffCode(uint32_t code){
    return (code == _offCode);
}

/**
  * Returns the proxy type PROXY_INPUT or PROXY_OUTPUT
  */
ProxyType RadioProxy::proxyType(){
    return _proxyType;
}

/**
  * Returns the pulse
  */
int RadioProxy::pulseLength(){
    return _pulseLength;
}

/**
  * Set the pulse length
  */
void RadioProxy::setPulseLength(int pulseLength){
    _pulseLength = pulseLength;
}

/**
  * Returns the protocol
  */
int RadioProxy::protocol(){
    return _protocol;
}

/**
  * Sets the protocol
  */
void RadioProxy::setProtocol(int protocol){
    _protocol = protocol;
}

/**
  * Returns the code length
  */
int RadioProxy::codeLength(){
    return _codeLength;
}

/**
  * Sets the code length
  */
void RadioProxy::setCodeLength(int codeLength){
    _codeLength = codeLength;
}

/**
  * Returns the number of transmit repetitions
  */
int RadioProxy::repetitions(){
    return _repetitions;
}

/**
  * Sets the number of transmit repetitions
  */
void RadioProxy::setRepetitions(int repetitions){
    _repetitions = repetitions;
}

/**
  * Returns the current status
  */
bool RadioProxy::state(){
    return _state;
}

/**
  * Sets the status
  */
void RadioProxy::setState(bool state){
    _state = state;
}

/**
  * Remove a proxy - this never happens as they are statically allocated globals
  */
void RadioProxy::removeProxy(RadioProxy* proxy){
    Serial.println("proxy deleted");
}

/**
  * Performs the mapping of a radio code to a WebThing property
  */
int RadioProxy::mapRadioStatus(){
  static uint32_t lastTime = 0;
  static uint32_t lastCode = 0;
  
  int onOff = -1;

  if (theRadio.available()) {
    uint32_t time = millis();
    uint32_t delta = time-lastTime;
    lastTime = time;
    int32_t code =  theRadio.getReceivedValue();
    if (code == lastCode && delta < RADIOPROXY_DEBOUNCE){
      // debounce - ignore same code within debounce time
      theRadio.resetAvailable();
      return -1;
    }
    lastCode = code;
    ThingPropertyValue value;  
    if (_verbose){
      Serial.print("Received code: ");
      Serial.print(code);
      Serial.print(", code length: ");
      Serial.print(theRadio.getReceivedBitlength());
      Serial.print(", pulse length: ");
      Serial.print(theRadio.getReceivedDelay());
      Serial.print(", protocol: ");
      Serial.print(theRadio.getReceivedProtocol()); 
    }
    RadioProxy* proxy = RadioProxy::getProxyForCode(code);
    if (proxy != nullptr){
      if (proxy->isFlipFlopCode(code)){
        // same code is used to turn on and off
        if (_verbose){
          Serial.print(", flip flop ");
        }        
        proxy->_state = !proxy->_state;
      }
      else if (proxy->isOnCode(code)){
        proxy->_state = 1;
        if (_verbose){
          Serial.print(", on code ");
        }
      }
      else if (proxy->isOffCode(code)){
        proxy->_state = 0;
        if (_verbose){
          Serial.print(", off code ");
        }
      }
      onOff = proxy->_state;
      value.boolean = proxy->_state;
      // has to be one or the other otherwise proxy would be null
      proxy->property()->setValue(value);
      if (_verbose){
        Serial.print("proxy found: ");
        ThingProperty* property = proxy->property();
        Serial.print(property->description);
        Serial.print(", new state: ");
        Serial.println(proxy->state());
      }
    }
    else{
      if (_verbose){
        Serial.println(", no proxy found");
      }
    }
    theRadio.resetAvailable();
  }
  return onOff;
}

/**
  * Maps all the current WebThings status to a radio device
  */
void RadioProxy::mapPropertyStatus(){
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    for (auto proxy : *proxies){
      if(proxy->proxyType() == PROXY_OUTPUT){
        mapPropertyStatus(proxy->property());
      }
    }
}

/**
  * Maps a single WebThing status to a radio device
  */
void RadioProxy::mapPropertyStatus(ThingProperty* property){
    
  ThingPropertyValue value = property->getValue();
  RadioProxy* proxy = RadioProxy::getProxyForProperty(property);
  if (proxy != nullptr){ 
    if ((proxy->isFlipFlopCode(proxy->onCode())) && (value.boolean != proxy->_state)){
        // same code is used to turn on and off - only do this if state has changed       
        RadioProxy::sendCodeToProxy(proxy, proxy->onCode());
        proxy->_state = !proxy->_state;
      }
    else if(value.boolean == 1 && proxy->state() != true){  
      RadioProxy::sendCodeToProxy(proxy, proxy->onCode());
      proxy->setState(true);
    }
    else if(value.boolean == 0 && proxy->state() != false){
      RadioProxy::sendCodeToProxy(proxy, proxy->offCode());
      proxy->setState(false);
    }
  }
}

/**
  * Sends a code to a proxy radio device
  */
void RadioProxy::sendCodeToProxy(RadioProxy* proxy, uint32_t code){
  if (_verbose){
    Serial.print("Sending code: ");
    Serial.print(code);
    Serial.print(", code length: "); 
    Serial.print(proxy->codeLength());
    Serial.print(", pulse length: ");    
    Serial.print(proxy->pulseLength());
    Serial.print(", protocol: ");
    Serial.print(proxy->protocol()); 
    Serial.print(", repetitions: "); 
    Serial.print(proxy->repetitions());
    Serial.print(", property: "); 
    ThingProperty* property = proxy->property();
    Serial.print(property->description);
    if (proxy->isFlipFlopCode(code)){
      Serial.println(", flip-flop code");
    } 
    else if (proxy->isOnCode(code)){
      Serial.println(", on code");
    }
    else if (proxy->isOffCode(code)){
      Serial.println(", off code");
    }  
  }

  theRadio.setProtocol(proxy->protocol()); // needs to be first - rest of data is in protocol structure
  theRadio.setPulseLength(proxy->pulseLength()); 
  theRadio.setRepeatTransmit(proxy->repetitions()); 
  theRadio.send(code, proxy->codeLength());
}

/**
  * Performs instance lookup to return the proxy for a code
  * otherwise returns nullptr
  */
RadioProxy* RadioProxy::getProxyForCode(uint32_t code){
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    for (auto proxy : *proxies){
      if((proxy->offCode() == code || proxy->onCode() == code) && proxy->proxyType() == PROXY_INPUT){  
        return proxy;
      }
    }
    return nullptr;
}

/**
  * Performs instance lookup to return the proxy for a WebThing property
  * otherwise returns nullptr
  */
RadioProxy* RadioProxy::getProxyForProperty(ThingProperty* property){        
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    for (auto proxy : *proxies){
      if(proxy->property() == property && proxy->proxyType() == PROXY_OUTPUT){
        return proxy;
      }
    }
    return nullptr;
}

/**
  * Holds a static list of all proxy instances added by the constructor
  * There are constructor timing issues if this is held globally rather
  * than in this class member function
  */
std::vector<RadioProxy*>* RadioProxy::getProxies(){
    static std::vector<RadioProxy*> proxies;
    return &proxies;
}



