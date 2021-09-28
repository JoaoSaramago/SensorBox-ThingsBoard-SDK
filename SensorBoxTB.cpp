#include "SensorBoxTB.h"

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #ifndef HAVE_HWSERIAL1
    #include "SoftwareSerial.h"
    SoftwareSerial Serial1(12, 13); // RX, TX
  #endif
#endif


String token_tb;

#if defined(ESP8266) || defined(ESP32)
  void writeStringEEPROM(int address, String word) {
    for (int i = 0; i < word.length(); ++i) {
      EEPROM.write(address + i, word[i]);
    }

    EEPROM.write(address + word.length(), '\0');
    EEPROM.commit();
  }

  String readStringEEPROM(int address) {
    String word;
    char readChar = ' ';
    int i = address;

    while (readChar != '\0') {
      readChar = char(EEPROM.read(i));
      delay(10);
      i++;

      if (readChar != '\0') {
        word += readChar;
      }
    }

    return word;
  }

  bool provisionRequestSent = false;
  volatile bool provisionResponseProcessed = false;

  void processProvisionResponse(const Provision_Data &data) {
    int jsonSize = measureJson(data) + 1;
    char buffer[jsonSize];
    serializeJson(data, buffer, jsonSize);
    Serial.println(buffer);
    if (strncmp(data["status"], "SUCCESS", strlen("SUCCESS")) != 0) {
      provisionResponseProcessed = true;
      Serial.println("[SensorBoxTB] Failed to provision device.");
      while(1);
    }
    if (strncmp(data["credentialsType"], "ACCESS_TOKEN", strlen("ACCESS_TOKEN")) == 0) {
      token_tb= data["credentialsValue"].as<String>();
      const char* rcv_token = data["credentialsValue"];
      writeStringEEPROM(0, "token");
      writeStringEEPROM(6, rcv_token);
    }
    provisionResponseProcessed = true;
  }

  const Provision_Callback provisionCallback = processProvisionResponse;

  void SensorBox::clearSavedToken() {
    EEPROM.begin(64);
    EEPROM.write(0, '\0');
    EEPROM.commit();
  }
#endif


void SensorBox::initComms() {

  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    Serial1.begin(WIFI_SHIELD_BAUD_RATE);
    WiFi.init(&Serial1);
    if (WiFi.status() == WL_NO_SHIELD) {
      #if SERIAL_DEBUG
        Serial.println(F("[SensorBoxTB] WiFi shield not present"));
      #endif
      return;
    }
  #endif

  #ifdef ESP32
  if (_username) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)_username, strlen(_username));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)_username, strlen(_username));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)_password, strlen(_password));
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
    esp_wifi_sta_wpa2_ent_enable(&config);
  }
  #endif

  #if defined(ESP8266) || defined(ESP32)
    EEPROM.begin(64);

    //Serial.print("address 0: "); Serial.println(EEPROM.readString(0));

    if (!_token && readStringEEPROM(0) == "token") {
      Serial.println("[SensorBoxTB] Found token in the EEPROM! Using it to connect...");
      token_tb = readStringEEPROM(6);
      Serial.print("[SensorBoxTB] EEPROM token: "); Serial.println(readStringEEPROM(6));
    } else if (_token) {
      token_tb= _token;
    }
  #else
    token_tb= _token;
  #endif


  while (!connected()) {
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED && strlen(token_tb.c_str()) > 0) {
      Serial.print("[SensorBoxTB] Using token: "); Serial.println(token_tb);
      connectThingsBoard();
    }
    #if defined(ESP8266) || defined(ESP32)
      else if (_provisionDeviceKey) {

        while (!provisionResponseProcessed) {
          if (!provisionRequestSent) {
            connectThingsBoardProvision();

            if (Provision_Subscribe(provisionCallback)) {
              if (sendProvisionRequest(_deviceName, _provisionDeviceKey, _provisionDeviceSecret)) {
                provisionRequestSent = true;
                Serial.println("[SensorBoxTB] Provision request was sent!");
              }
            }
          }
          ThingsBoard::loop();
        }
        if (connected()) {
          disconnect();
        }
        ThingsBoard::loop();
      }
    #endif

  }
}



void SensorBox::connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    subscribedRPC = false;

    #if SERIAL_DEBUG
      Serial.print(F("[SensorBoxTB] Connecting to WiFi"));
    #endif

    if (WiFi.status() != WL_CONNECTED) {
      #ifdef ESP32
      if (_username) {
        WiFi.begin(_ap_name);
      } else {
        WiFi.begin(_ap_name, _password);
      }
      #else
        WiFi.begin(_ap_name, _password);
      #endif

      for (int i = 0; i <= 10 && ( WiFi.status() == WL_DISCONNECTED || WiFi.status() == WL_IDLE_STATUS ); i++) {
        #if SERIAL_DEBUG
          Serial.print(".");
        #endif
        delay(1000);
      }
    }
    Serial.print("\n");
    #if SERIAL_DEBUG
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(F("[SensorBoxTB] Connected to WiFi!"));
    } else {
      Serial.println(F("[SensorBoxTB] Not connected to WiFi."));
    }
    #endif
  }
}


#if defined(ESP8266) || defined(ESP32)
  boolean SensorBox::connectThingsBoardProvision() {
    if (!connected() && WiFi.status() == WL_CONNECTED) {
      subscribedRPC = false;
      #if SERIAL_DEBUG
        Serial.println(F("[SensorBoxTB] Connecting to ThingsBoard Provison node ..."));
      #endif

      if ( !connect(_server_ip, "provision") ) {
        #if SERIAL_DEBUG
          Serial.print(F("[SensorBoxTB] Failed to connect to ThingsBoard Provision node."));
        #endif
        return false;
      }

      #if SERIAL_DEBUG
        Serial.println(F("[SensorBoxTB] Connected successfully to ThingsBoard Provsion node!"));
      #endif
    }
    return true;
  }
#endif


boolean SensorBox::connectThingsBoard() {
  if (!connected() && WiFi.status() == WL_CONNECTED) {
    subscribedRPC = false;
    #if SERIAL_DEBUG
      Serial.println(F("[SensorBoxTB] Connecting to ThingsBoard node ..."));
    #endif

    if ( !connect(_server_ip, token_tb.c_str()) ) {
      #if SERIAL_DEBUG
        Serial.println(F("[SensorBoxTB] Failed to connect to ThingsBoard node."));
      #endif
      return false;
    }

    #if SERIAL_DEBUG
      Serial.println(F("[SensorBoxTB] Connected successfully to ThingsBoard node!"));
    #endif
  }
  return true;
}


#if defined(ESP8266) || defined(ESP32)
  boolean SensorBox::subscribeActuatorsCmds(RPC_Callback *callbacks, size_t callbacks_size) {
    if(callbacks && !subscribedRPC && connected()) {
      if (!_callbacks) {
        _callbacks = callbacks;
        _callbacks_size = callbacks_size;
      }
      #if SERIAL_DEBUG
        Serial.print(F("[SensorBoxTB] Subscribing to ")); Serial.print(callbacks_size); Serial.println(F(" actuators commands ..."));
      #endif

      if (!RPC_Subscribe(callbacks, callbacks_size)) {
        #if SERIAL_DEBUG
          Serial.print(F("[SensorBoxTB] Failed to subscribe to Actuators."));
        #endif
        return false;
      }
      subscribedRPC = true;
      #if SERIAL_DEBUG
        Serial.println(F("[SensorBoxTB] Subscribed successfully to Actuators!"));
      #endif
    }
    return true;
  }
#endif


boolean SensorBox::sendDeviceDetails(DeviceDetails *details, size_t details_size) {
  #if SERIAL_DEBUG
    Serial.println(F("[SensorBoxTB] Sending device's details ..."));
  #endif

  for (size_t i = 0; i < details_size; ++i) {
    sendAttributeString(details[i].m_name, details[i].m_units);
    #if SERIAL_DEBUG
      Serial.print(F("[SensorBoxTB]  - Sent ")); Serial.print(details[i].m_name); Serial.println(F(" details."));
    #endif
  }

  return true;
}

#if defined(ESP8266) || defined(ESP32)
  void SensorBox::checkFirmwareUpdates(const char* currFwTitle, const char* currFwVersion) {
    Serial.println("[SensorBoxTB] Checking Firwmare Updates...");
    Firmware_OTA_Subscribe();
    if (Firmware_Update(currFwTitle, currFwVersion)) {
      Serial.println("[SensorBoxTB] Update done, reboot now");
  #if defined(ESP8266)
      ESP.restart();
  #elif defined(ESP32)
      esp_restart();
  #endif
    }
    else {
      Serial.println("[SensorBoxTB] No new firmware");
    }
    Firmware_OTA_Unsubscribe();
  }
#endif



unsigned long last_time_tb_loop;

void SensorBox::loop() {

  if ( millis() - last_time_tb_loop > 5000 ) {
    connectWiFi();
    connectThingsBoard();
    #if defined(ESP8266) || defined(ESP32)
      subscribeActuatorsCmds(_callbacks, _callbacks_size);
    #endif

    last_time_tb_loop = millis();
  }

  ThingsBoard::loop();

}
