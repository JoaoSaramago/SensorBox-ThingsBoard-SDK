#include <SensorBoxTB.h>

const char* ssid               = "ssid";
const char* password           = "password";

const char* thingsboard_server = "thingsboard.rnl.tecnico.ulisboa.pt";
const char* token              = "LAB3_ATUADORES";

const int tempLedPin = 23;
const int potLedPin = 22;
const int lightLedPin = 21;

// PWM setup
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

unsigned long prevBlinkCheck = 0;

int blinkInterval = 200;

RPC_Response setLedTempAlarm(const RPC_Data &data) {
  bool ledTempState = data["value"];
  digitalWrite(tempLedPin, ledTempState);
  return RPC_Response("yellow_led_alarm", ledTempState);
}

RPC_Response setLedIntensity(const RPC_Data &data) {
  int intensity = data["value"];
  ledcWrite(pwmChannel, intensity);
  return RPC_Response("blue_led_intensity", intensity);
}

RPC_Response setLedBlinkInterval(const RPC_Data &data) {
  blinkInterval = data["value"];
  return RPC_Response("red_led_blink_interval", blinkInterval);
}

RPC_Callback callbacks[] = {
  {"setLedTempAlarm", setLedTempAlarm},
  {"setLedIntensity", setLedIntensity},
  {"setLedBlinkInterval", setLedBlinkInterval},
};

SensorBox sb(ssid, password, thingsboard_server, token);

void setup() {
  Serial.begin(115200);
  
  pinMode(tempLedPin, OUTPUT);
  pinMode(potLedPin, OUTPUT);
  pinMode(lightLedPin, OUTPUT);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(lightLedPin, pwmChannel);

  sb.initComms();
  sb.subscribeActuatorsCmds(callbacks, COUNT_OF(callbacks));
} 

void loop() {
  setLedBlinkIntervalTask();
  sb.loop();
}

void setLedBlinkIntervalTask() {
  if (millis() - prevBlinkCheck > blinkInterval) {
    prevBlinkCheck = millis();
    digitalWrite(potLedPin, LOW);
  } else if (millis() - prevBlinkCheck > blinkInterval/2) {
    digitalWrite(potLedPin, HIGH);
  }
}
