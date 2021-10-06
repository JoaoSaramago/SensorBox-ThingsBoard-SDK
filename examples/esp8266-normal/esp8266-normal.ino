#include <SensorBoxTB.h>
#include <Wire.h>
#include <SHTSensor.h>

#define SOUND_VELOCITY 0.034

#define SERIAL_DEBUG_BAUD   115200

#define WIFI_AP_NAME        "wi-fi_ap_name"
#define WIFI_PASSWORD       "wi-fi_password"

#define TOKEN               "ESP8266_TOKEN"
#define THINGSBOARD_SERVER  "thingsboard.rnl.tecnico.ulisboa.pt" 

#define TEMP_HUMID_TASK_DELAY 5000  // ms
#define TEMP_THRESHOLD_SEND 0.1  // ºC

#define LIGHT_TASK_DELAY 100  // ms
#define LIGHT_THRESHOLD_SEND 60  // mV
#define LIGHT_SENSOR_PIN A0  

#define NOISE_TASK_DELAY 50  // ms
#define NOISE_DELAY_OFF 3000  // ms
#define NOISE_SENSOR_PIN D7  

#define PRESENCE_TASK_DELAY 500  // ms
#define PRESENCE_DETECTED_DISTANCE 150  // cm
#define PRESENCE_TRIGGER_PIN D6  
#define PRESENCE_ECHO_PIN D5 

#define BUZZER_TASK_DELAY 1000  // ms
#define BUZZER_PIN D8  

#define LEDS_TASK_DELAY 100  // ms
#define INTERNET_LED_PIN D4  
#define THINGSBOARD_LED_PIN D3 
#define MESSAGE_SENT_LED_PIN D0  


// Para o sensor de temperatura e humidade
SHTSensor sht;
float last_temperature;
int last_humidity;
unsigned long last_time_temperature_humidity;

// Para o sensor de intensidade de luz
int last_light_intensity;
unsigned long last_time_light_intensity;

// Para o sensor de ruido
bool last_sent_noise;
unsigned long last_time_sent_noise;
unsigned long noise_delay_off_started;

// Para o sensor de presenca
bool last_sent_presence;
unsigned long last_time_presence;

// For buzzer
bool buzzer_on = false;
unsigned long last_time_buzzer;

// Para os LEDs de estado
int message_sent = false;
unsigned long last_time_leds_task;


// Processa a chamada RPC "setBuzzer", aciona a campainha consoante o comando recebido
RPC_Response processSetBuzzer(const RPC_Data &data) {
  buzzer_on = data["value"];
  digitalWrite(BUZZER_PIN, buzzer_on);
  
  return RPC_Response("buzzer", buzzer_on);
}

// Lista de relacao entre as chamadas de RPC e as funcoes que as processam
RPC_Callback callbacks[] = {
  {"setBuzzer", processSetBuzzer},
};

// Detalhes sobre os sensores e atuadores usados, unidades em que sao enviados e recebidos os valores
DeviceDetails details[] = {
  { "temperature",     "ºC" },
  { "humidity",        "%" },
  { "light_intensity", "mV" },
  { "noise",           "boolean" },
  { "presence",        "boolean" },
  { "buzzer",          "{value: boolean}" },
};

// Inicializar o objeto SensorBox passando os dados necessários para conexao a rede Wi-Fi WP2-PSK e ao ThingsBoard
SensorBox sb(WIFI_AP_NAME, WIFI_PASSWORD, THINGSBOARD_SERVER, TOKEN);


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  
  // Inicializar o pino do sensor de intensidade de luz
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  
  // Inicializar o pino do sensor de ruido
  pinMode(NOISE_SENSOR_PIN, INPUT);
  
  // Inicializar os pinos do sensor de presenca
  pinMode(PRESENCE_TRIGGER_PIN, OUTPUT);
  pinMode(PRESENCE_ECHO_PIN, INPUT); 
  
  // Inicializar o pino do atuador campainha
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Inicializar os pinos dos LEDs de estado
  pinMode(INTERNET_LED_PIN, OUTPUT);
  pinMode(THINGSBOARD_LED_PIN, OUTPUT);
  pinMode(MESSAGE_SENT_LED_PIN, OUTPUT);

  // Inicializar comunicacoes I2C com o sensor de temperatura e humidade
  Wire.begin();
  temp_humid_sensor_init();

  // Estabelecer comunicacao com a rede Wi-Fi e o servidor do ThingsBoard
  sb.initComms();
  // Enviar os detalhes dos sensores e atuadores deste dispositivo
  sb.sendDeviceDetails(details, COUNT_OF(details));
  // Subscrever as mensagens das chamadas RPC que acionam os atuadores
  sb.subscribeActuatorsCmds(callbacks, COUNT_OF(callbacks)); 
}


void loop() {
  leds_task();
  
  // Apenas corre o codigo relacionado com os sensores e atuadores se estiver ligado ao servidor
  if (sb.connected()) {
    temperature_humidity_task();
    light_sensor_task();
    noise_task();
    presence_task();
    buzzer_task();
  }

  // Manutenção das comunicacoes
  sb.loop();
}


// Verificar se o estados dos LEDs devem ser alterados, caso precisem altera de acordo com a logica implementada
void leds_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_leds_task < LEDS_TASK_DELAY ) {
    return;
  }

  // Wi-Fi conectado?
  if (WiFi.status() != WL_CONNECTED) {  // WiFi connected
    digitalWrite(INTERNET_LED_PIN, LOW);
  } else {
    digitalWrite(INTERNET_LED_PIN, HIGH);
  }

  // ThingsBoard conectado?
  if (!sb.connected()) {  // thingsboard connected
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


// Inicializa o sensor de temperatura e humidade
void temp_humid_sensor_init() {
  while (!sht.init()) {
    Serial.println("Temperature/Humidity sensor not found");
    delay(500);
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
}


// Fazer a leitura do sensor de temperatura e humidade e enviar se necessario
void temperature_humidity_task() {
  bool update_temp_hum = millis() - last_time_temperature_humidity > TEMP_HUMID_TASK_DELAY;

  // Se estiver na altura de fazer nova leitura do sensor e se a leitura for feita com sucesso, entra
  if ( update_temp_hum && sht.readSample() ) {
  
    float temperature = sht.getTemperature();

    if (isnan(temperature)) {
      Serial.println("Failed to read from temperature sensor!");
    } else {
      // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
      if (abs(last_temperature - temperature) > TEMP_THRESHOLD_SEND || isnan(last_temperature)) {
          Serial.print("Sending Temperature...");
          Serial.println(temperature);
          // Envia a temperatura como float e arredonda a uma casa decimal
          sb.sendTelemetryFloat("temperature", roundf(temperature * 10.0) / 10.0);
          last_temperature = temperature;
          message_sent = true;
      }
    }

    int humidity = sht.getHumidity();

    if (isnan(humidity)) {
      Serial.println("Failed to read from humidity sensor!");
    } else {
      // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
      if (last_humidity != humidity || isnan(last_humidity)) {
          Serial.print("Sending humidity...");
          Serial.println(humidity);
          // Envia a humidade como inteiro
          sb.sendTelemetryInt("humidity", humidity);
          last_humidity = humidity;
          message_sent = true;
      }
    }
    
  }
  last_time_temperature_humidity = millis();
}


// Fazer a leitura de intensidade da luz e enviar se necessario
void light_sensor_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_light_intensity < LIGHT_TASK_DELAY ) { 
    return;
  }
  
  int light_intensity = analogRead(LIGHT_SENSOR_PIN) * 3.3;  // convert 0-1V to 0-3.3V

  if (isnan(light_intensity)) {
    Serial.println("Failed to read from light intensity sensor!");
  } else {
    // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
    if (abs(last_light_intensity - light_intensity) > LIGHT_THRESHOLD_SEND || isnan(last_light_intensity)) {
        Serial.print("Sending Light Intensity...");
        Serial.println(light_intensity);
        // Envia a intensidade da luz como inteiro
        sb.sendTelemetryInt("light_intensity", light_intensity);
        last_light_intensity = light_intensity;
        message_sent = true;
    }
  }

  last_time_light_intensity = millis();
}


// Fazer a leitura do sensor de ruido e enviar se necessario
void noise_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_sent_noise < NOISE_TASK_DELAY ) {
    return;
  }
  
  bool noise = digitalRead(NOISE_SENSOR_PIN); 

  if (isnan(noise)) {
    Serial.println("Failed to read from sound sensor!");
  } else { 
    // Envia se o estado do ruido for diferente do ultimo enviado e o atraso entre mudanças de estados já tiver sido cumprido, ou se ainda nao tiver enviado nada     
    if (last_sent_noise != noise && (millis() - noise_delay_off_started > NOISE_DELAY_OFF ) || isnan(last_sent_noise)) {
      
      Serial.print("Sending Noise state...");
      Serial.println(noise);
      sb.sendTelemetryBool("noise", noise);
      last_sent_noise = noise;
      message_sent = true;

      // Se for detetado barulho reinicia o contador do atraso para sair do estado que diz haver ruido
      if (noise) {
        noise_delay_off_started = millis();
      }
    // Se for detado ruido inicia o contador que bloqueia este estado durante um tempo predefinido
    } else if (noise) {
      noise_delay_off_started = millis();
    }
  }
  
  last_time_sent_noise = millis();
}


// Fazer a leitura do sensor de presenca e enviar se necessario
void presence_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_presence < PRESENCE_TASK_DELAY ) {
    return;
  }

  digitalWrite(PRESENCE_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PRESENCE_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PRESENCE_TRIGGER_PIN, LOW);

  long duration = pulseIn(PRESENCE_ECHO_PIN, HIGH);
  float distance = duration * SOUND_VELOCITY/2;

  // Serial.print("Distance (cm): "); Serial.println(distance);
  
  bool presence = (distance < PRESENCE_DETECTED_DISTANCE) ? true : false;

  // Envia se o estado de presenca for diferente do ultimo enviado, ou se ainda nao tiver enviado nenhum
  if (last_sent_presence != presence || isnan(last_sent_presence)) {
      Serial.print("Sending Presence state...");
      Serial.println(presence);
      sb.sendTelemetryBool("presence", presence);
      last_sent_presence = presence;
      message_sent = true;
  }

  last_time_presence = millis();
}


// Acionar a campainha consoante o estado definido e caso a campainha esteja ativada faz ela tocar de forma intermitente
void buzzer_task() {
  // Verifica se está na altura de executar esta função, se ainda não estiver sai imediatamente
  if ( millis() - last_time_buzzer < BUZZER_TASK_DELAY ) {
    return;
  }

  if (buzzer_on) {
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
  } else if (digitalRead(BUZZER_PIN)) {
    digitalWrite(BUZZER_PIN, LOW);
  }

  last_time_buzzer = millis();
}
