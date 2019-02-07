#include <WebThingAdapter.h>
#include <RCSwitch.h>

enum ProxyType { PROXY_INPUT, PROXY_OUTPUT };

class RadioProxy{
 
   public:
    RadioProxy(ProxyType proxyType, ThingProperty* property, uint32_t onCode, uint32_t offCode);
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
    void setPulseLength(int pulseLength);
    void setProtocol(int protocol);
    void setCodeLength(int codeLength);
    bool status();
  
    static void mapPropertyStatus(ThingProperty* property);
    static int mapRadioStatus();
    static void enableRadio(int pin = 4, int pulseLength = 186);
  
  private:
    ProxyType _proxyType;
    ThingProperty* _property;
    uint32_t _offCode;
    uint32_t _onCode;
    int _pulseLength;
    int _codeLength;
    int _protocol;
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