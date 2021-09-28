#ifndef SENSOR_BOX_TB_H
#define SENSOR_BOX_TB_H

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define SERIAL_DEBUG true

#include <Arduino.h>
#include <ThingsBoard.h>

#if defined(ESP8266)

  #pragma message "Compiling for ESP8266."
  #include <ESP8266WiFi.h>
  #include <EEPROM.h>

#elif defined(ESP32)

  #pragma message "Compiling for ESP32."
  #include <WiFi.h>
  #include <EEPROM.h>

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

  #pragma message "Compiling for Arduino Uno."
  #include <WiFiEspAT.h>

  #define WIFI_SHIELD_BAUD_RATE 9600

#else

  #error "This device is not supported, use an ESP32, ESP8266 or Arduino."

#endif


class DeviceDetails {

  public:

    inline DeviceDetails():m_name(), m_units() {  }

    inline DeviceDetails(const char *deviceName, const char *units):m_name(deviceName), m_units(units) {  }

    const char  *m_name;
    const char  *m_units;

};


class SensorBox: public ThingsBoard {

  public:

    SensorBox(const char *ap_name, const char *password, const char *server_ip, const char *token) : ThingsBoard(wifi_client) {
      _ap_name = ap_name;
      _password = password;
      _server_ip = server_ip;
      _token = token;
    }

    SensorBox(char *ap_name, char *password, char *server_ip, const char* deviceName, const char* provisionDeviceKey, const char* provisionDeviceSecret) : ThingsBoard(wifi_client) {
      _ap_name = ap_name;
      _password = password;
      _server_ip = server_ip;
      _deviceName = deviceName;
      _provisionDeviceKey = provisionDeviceKey;
      _provisionDeviceSecret = provisionDeviceSecret;
    }

    ~SensorBox() {}

    void provision();

    void initComms();

    void connectWiFi();

    boolean connectThingsBoardProvision();

    boolean connectThingsBoard();

    boolean subscribeActuatorsCmds(RPC_Callback *callbacks, size_t callbacks_size);

    boolean sendDeviceDetails(DeviceDetails *details, size_t details_size);

    void checkFirmwareUpdates(const char* currFwTitle, const char* currFwVersion);

    void clearSavedToken();

    void loop();

  private:
    #if defined(ESP8266)
      WiFiClient wifi_client;
    #elif defined(ESP32)
      WiFiClient wifi_client;
    #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
      // WiFiEspClient wifi_client;
      WiFiClient wifi_client;
    #endif

    int status  = WL_IDLE_STATUS;

    bool subscribedRPC = false;

    const char *_ap_name;
    const char *_password;
    const char *_server_ip;

    const char * _token;

    const char* _deviceName;
    const char* _provisionDeviceKey;
    const char* _provisionDeviceSecret;

    RPC_Callback *_callbacks;
    size_t _callbacks_size;
};

#endif
