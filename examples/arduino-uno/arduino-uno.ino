#include <SensorBoxTB.h>

#define SERIAL_DEBUG_BAUD    115200

#define WIFI_AP_NAME        "wi-fi_ap_name"
#define WIFI_PASSWORD       "wi-fi_password"

#define TOKEN               "ARDUINO_UNO_TOKEN"
#define THINGSBOARD_SERVER  "thingsboard.rnl.tecnico.ulisboa.pt" 

#define LIGHT_TASK_DELAY 200  // ms
#define LIGHT_THRESHOLD_SEND 60  // mV
#define LIGHT_SENSOR_PIN A0  

#define TEMP_TASK_DELAY 1000  // ms
#define TEMP_THRESHOLD_SEND 5  // ºC
#define TEMP_SENSOR_PIN A2  

#define LEDS_TASK_DELAY 100  // ms
#define INTERNET_LED_PIN 4  
#define THINGSBOARD_LED_PIN 3 
#define MESSAGE_SENT_LED_PIN 2  


// Para o sensor de intensidade de luz
int last_light_intensity;
unsigned long last_time_light_intensity;

// Para o sensor de temperatura
float last_temperature;
unsigned long last_time_temperature;

// Para os LEDs de estado
bool message_sent = false;
unsigned long last_time_leds_task;

// Detalhes sobre os sensores usados, unidades em que sao enviados os valores
const DeviceDetails details[] = {
  { "light_intensity", "mV" },
  { "temperature", "ºC" },
};

// Inicializar o objeto SensorBox passando os dados necessários para conexao a rede Wi-Fi WP2-PSK e ao ThingsBoard
SensorBox sb(WIFI_AP_NAME, WIFI_PASSWORD, THINGSBOARD_SERVER, TOKEN);


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  
  // Inicializar o pino do sensor de intensidade de luz
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  
  // Inicializar o pino do sensor de temperatura
  pinMode(TEMP_SENSOR_PIN, INPUT);

  // Inicializar os pinos dos LEDs de estado
  pinMode(INTERNET_LED_PIN, OUTPUT);
  pinMode(THINGSBOARD_LED_PIN, OUTPUT);
  pinMode(MESSAGE_SENT_LED_PIN, OUTPUT);

  // Estabelecer comunicacao com a rede Wi-Fi e o servidor do ThingsBoard
  sb.initComms();
  // Enviar os detalhes dos sensores deste dispositivo
  sb.sendDeviceDetails(details, COUNT_OF(details));
}


void loop() { 
  leds_task(); 
  
  // Apenas corre o codigo relacionado com os sensores se estiver ligado ao servidor
  if (sb.connected()) {
    light_sensor_task();
    temperature_task();
  }

  // Manutencao das comunicacoes
  sb.loop();
}


// Verificar se o estados dos LEDs devem ser alterados, caso precisem altera de acordo com a logica implementada
void leds_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_leds_task < LEDS_TASK_DELAY ) {
    return;
  }

  // Wi-Fi conectado?
  if (WiFi.status() != WL_CONNECTED) {  
    digitalWrite(INTERNET_LED_PIN, LOW);
  } else {
    digitalWrite(INTERNET_LED_PIN, HIGH);
  }

  // ThingsBoard conectado?
  if (!sb.connected()) {  
    digitalWrite(THINGSBOARD_LED_PIN, LOW);
  } else {
    digitalWrite(THINGSBOARD_LED_PIN, HIGH);
  }

  // Mensagem enviada?
  if (message_sent) {  
    digitalWrite(MESSAGE_SENT_LED_PIN, HIGH);
    message_sent = false;
  } else {
    digitalWrite(MESSAGE_SENT_LED_PIN, LOW);
  }

  last_time_leds_task = millis();
}


// Fazer a leitura do sensor de intensidade de luz e enviar se necessario
void light_sensor_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_light_intensity < LIGHT_TASK_DELAY ) {
    return;
  }
 
  int light_intensity = analogRead(LIGHT_SENSOR_PIN);

  if (isnan(light_intensity)) {
    Serial.println(F("Failed to get light intensity value!"));
  } else {
    // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
    if (abs(last_light_intensity - light_intensity) > LIGHT_THRESHOLD_SEND || isnan(last_light_intensity)) {
        // Envia a intensidade da luz como inteiro
        sb.sendTelemetryInt("light_intensity", light_intensity);
        Serial.print(F("Sent Light Intensity:")); Serial.println(light_intensity);
        message_sent = true;
        last_light_intensity = light_intensity;
    }
  }

  last_time_light_intensity = millis();
}


// Fazer a leitura do sensor de temperatura e enviar se necessario
void temperature_task() {
  if ( millis() - last_time_temperature < TEMP_TASK_DELAY ) { // Update and send only each X milliseconds
    return;
  }
  
  float temperature = (analogRead(TEMP_SENSOR_PIN) * 5.0) / 1024.0;  //multiply by 5V to get voltage
  temperature = (temperature - 0.5) * 100;
  temperature = roundf(temperature * 10.0) / 10.0;

  if (isnan(temperature)) {
    Serial.println(F("Failed to get temperature value!"));
  } else {
    // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
    if (abs(last_temperature - temperature) > TEMP_THRESHOLD_SEND || isnan(last_temperature)) {
        // Envia a temperatura como float
        sb.sendTelemetryFloat("temperature", temperature);
        Serial.print(F("Sent Temperature:")); Serial.println(temperature);
        message_sent = true;
        last_temperature = temperature;
    }
  }
  
  last_time_temperature = millis();
}
