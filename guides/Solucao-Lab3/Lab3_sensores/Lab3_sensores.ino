#include <SensorBoxTB.h>

const char* ssid               = "ssid";
const char* password           = "password";

const char* thingsboard_server = "thingsboard.rnl.tecnico.ulisboa.pt";
const char* token              = "LAB3_SENSORES";

const int tempSensorPin = 34;
const int lightSensorPin = 35;
const int potSensorPin = 32;

// Calibration
const float tempOffset = 0.0;
const int potMin = 0;
const int potMax = 4095;
const int lightSensorMin = 0;
const int lightSensorMax = 4095;

// Multi-Sampling
const int n_samples = 20;

const int tempTaskDelay = 1500;
const int lightTaskDelay = 700;
const int blinkTaskDelay = 500;

unsigned long lastTimeTemp = 0;
unsigned long lastTimeLight = 0;
unsigned long lastTimeBlink = 0;

SensorBox sb(ssid, password, thingsboard_server, token);

void setup() {
  Serial.begin(115200);
  sb.initComms();
} 

void loop() {
  readBlinkIntervalTask();
  readLightTask();
  readTempTask();
  sb.loop();
}

void readTempTask() {
  if ( millis() - lastTimeTemp < tempTaskDelay ) {
    return;
  }
  
  int reading = multiSampling(tempSensorPin);
  float temperature = (((reading / 4096.0) * 3.3) - 0.5) *100;
  temperature = temperature + tempOffset;
  temperature = roundf(temperature * 10) / 10;  // Round to one decimal place
  Serial.print("Temperature: "); Serial.println(temperature);
  sb.sendTelemetryFloat("temperature", temperature);
  lastTimeTemp = millis();
}

void readLightTask() {
  if ( millis() - lastTimeLight < lightTaskDelay ) {
    return;
  }
  
  int reading = multiSampling(lightSensorPin);
  int intensity = map(reading, lightSensorMin, lightSensorMax, 0, 255);
  Serial.print("Intensity: "); Serial.println(intensity);
  sb.sendTelemetryInt("light_intensity", intensity);
  lastTimeLight = millis();
}

void readBlinkIntervalTask() {
  if ( millis() - lastTimeBlink < blinkTaskDelay ) {
    return;
  }
    
  int reading = multiSampling(potSensorPin);
  int readBlinkInterval = map(reading, potMin, potMax, 200, 2000);
  Serial.print("Read Blink Interval (ms): "); Serial.println(readBlinkInterval);
  sb.sendTelemetryInt("blink_interval", readBlinkInterval);
  lastTimeBlink = millis();
}

// Measures da analog pin n_samples times and returns the mean of those readings
int multiSampling(int pin) {
  int sum = 0;
  int reading = 0;
  for (int i=0; i<n_samples; i++){
    reading = analogRead(pin);
    sum += reading;
  }
  return sum / n_samples;
}
