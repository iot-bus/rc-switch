#include "RadioProxy.h"

int RadioProxy::proxyCount = 0;

RCSwitch RadioProxy::theRadio;
bool RadioProxy::radioEnabled = 0;

RadioProxy::RadioProxy(ProxyType proxyType, ThingProperty* property, uint32_t onCode, uint32_t offCode){   
    _proxyType = proxyType;
    _property = property;
    _onCode = onCode;
    _offCode = offCode;
    _status = false;
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    proxies->push_back(this);
    proxyCount++;
}

RadioProxy::~RadioProxy(){
    removeProxy(this);
}

void RadioProxy::enableRadio(int radioPin, int pulseLength, int repetitions){
    if(!radioEnabled){
        // enable the radio
        theRadio.enableReceive(radioPin);
        theRadio.setPulseLength(pulseLength);
        theRadio.setRepeatTransmit(repetitions);
        radioEnabled = true;
    }
}

ThingProperty* RadioProxy::property(){
    return _property;
}

uint32_t RadioProxy::offCode(){
    return _offCode;
}

uint32_t RadioProxy::onCode(){
    return _onCode;
}

bool RadioProxy::isOnCode(uint32_t code){
    return (code == _onCode);
}

bool RadioProxy::isOffCode(uint32_t code){
    return (code == _offCode);
}

ProxyType RadioProxy::proxyType(){
    return _proxyType;
}

int RadioProxy::pulseLength(){
    return _pulseLength;
}

int RadioProxy::protocol(){
    return _protocol;
}

void RadioProxy::setPulseLength(int pulseLength){
    _pulseLength = pulseLength;
}

void RadioProxy::setCodeLength(int codeLength){
    _codeLength = codeLength;
}

void RadioProxy::setProtocol(int protocol){
    _protocol = protocol;
}

bool RadioProxy::status(){
    return _status;
}

void RadioProxy::setStatus(bool status){
    _status = status;
}

void RadioProxy::removeProxy(RadioProxy* proxy){
    // TO DO remove proxy from instance list
    Serial.println("proxy deleted");
}

int RadioProxy::mapRadioStatus(){
  ThingPropertyValue value;  
  int onOff = -1;
  if (theRadio.available()) {
    int32_t code =  theRadio.getReceivedValue();
    Serial.print("Received code: ");
    Serial.println(code);
    
    RadioProxy* proxy = RadioProxy::getProxyForCode(code);
    if (proxy != nullptr){
      if (proxy->isOnCode(code)){
        value.boolean = 1;
      }
      else if (proxy->isOffCode(code)){
        value.boolean = 0;
      }
      onOff = value.boolean;
      // has to be one or the other otherwise proxy would be null
      proxy->property()->setValue(value);
    }
    theRadio.resetAvailable();
  }
  return onOff;
}

void RadioProxy::mapPropertyStatus(){
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    for (auto proxy : *proxies){
      if(proxy->proxyType() == PROXY_OUTPUT){
        mapPropertyStatus(proxy->property());
      }
    }
}

void RadioProxy::mapPropertyStatus(ThingProperty* property){
    
  ThingPropertyValue value = property->getValue();
  RadioProxy* proxy = RadioProxy::getProxyForProperty(property);
  if (proxy != nullptr){ 
    if(value.boolean == 1 && proxy->status() != true){   
      theRadio.send(proxy->onCode(), 24);
      Serial.print("Sending code ");
      Serial.println(proxy->onCode());
      proxy->setStatus(true);
    }
    else if(value.boolean == 0 && proxy->status() != false){
      theRadio.send(proxy->offCode(), 24);
      Serial.print("Sending code ");
      Serial.println(proxy->offCode());
      proxy->setStatus(false);
    }
  }
}

RadioProxy* RadioProxy::getProxyForCode(uint32_t code){
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    for (auto proxy : *proxies){
      if((proxy->offCode() == code || proxy->onCode() == code) && proxy->proxyType() == PROXY_INPUT){  
        return proxy;
      }
    }
    return nullptr;
}

RadioProxy* RadioProxy::getProxyForProperty(ThingProperty* property){        
    std::vector<RadioProxy*>* proxies = RadioProxy::getProxies();
    for (auto proxy : *proxies){
      if(proxy->property() == property && proxy->proxyType() == PROXY_OUTPUT){
        return proxy;
      }
    }
    return nullptr;
}

std::vector<RadioProxy*>* RadioProxy::getProxies(){
    static std::vector<RadioProxy*> proxies;
    return &proxies;
}



