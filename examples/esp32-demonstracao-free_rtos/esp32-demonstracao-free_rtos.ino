#include <SensorBoxTB.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// Para o sensor de temperatura e humidade
#include "SHTSensor.h"

// Para o sensor de luminosidade
#include <BH1750.h>

// Para o sensor de TVOC e CO2
#include "sensirion_common.h"
#include "sgp30.h"

// Titulo e versão deste Firmware
#define CURRENT_FIRMWARE_TITLE    "SensorBox Demonstracao"
#define CURRENT_FIRMWARE_VERSION  "1.0"

// Dados para o provisionamento do dispositivo
#define PROVISION_DEVICE_KEY     "alunos-provisioning-key"
#define PROVISION_DEVICE_SECRET  "alunos-provisioning-secret"
#define DEVICE_NAME  "ESP32 Provisionado"

#define SERIAL_DEBUG_BAUD    115200

#define WIFI_AP_NAME        "wi-fi_ap_name"
#define WIFI_PASSWORD       "wi-fi_password"

#define TOKEN               "ESP32_TOKEN"
#define THINGSBOARD_SERVER  "thingsboard.rnl.tecnico.ulisboa.pt" 

#define BUZZER_STATE_TIME  1000  // ms
#define BUZZER_PIN 13  

#define TEMP_HUMID_TASK_DELAY 3000  // ms
#define TEMP_THRESHOLD_SEND 0.1  // ºC

#define LIGHT_TASK_DELAY 1000  // ms
#define LIGHT_THRESHOLD_SEND 1  // lux

#define TVOC_TASK_DELAY 5000  // ms
#define TVOC_THRESHOLD_SEND 1  // ppb
#define CO2_THRESHOLD_SEND 1  // ppb

#define NOISE_TASK_DELAY 30  // ms
#define NOISE_DELAY_OFF 3000  // ms
#define NOISE_SENSOR_PIN 34

#define MOTION_TASK_DELAY 100  // ms
#define MOTION_SENSOR_PIN 27  

#define LED_STRIP_TASK_DELAY 100  // ms
#define INTERNET_LED 7  
#define THINGSBOARD_LED 6 
#define TEMP_HUMID_LED 5
#define LIGHT_INTENSITY_LED 4  
#define TVOC_CO2_LED 3
#define NOISE_LED 2
#define MOTION_LED 1
#define BUZZER_LED 0   

// Configuracao da fita de LEDs
#define PIN       32
#define NUMPIXELS 8

// Inicializar o objeto que representa a fita de LEDs
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Para o sensor de temperatura e humidade
SHTSensor sht;

// Para o sensor de luminosidade
BH1750 light_meter;

// Para a campainha
bool buzzer_on = false;

// Para a fita de LEDs de estado
bool internet = false;
bool thingsboard = false;
bool sent_temp_humid = false;
bool sent_light_intensity = false;
bool sent_tvoc_co2 = false;
bool sent_noise = false;
bool sent_motion = false;

bool sb_connected = false;


// Processa a chamada RPC "setBuzzer", aciona a campainha consoante o comando recebido
RPC_Response processSetBuzzer(const RPC_Data &data)

  buzzer_on = data["value"];
  digitalWrite(BUZZER_PIN, buzzer_on);
  
  return RPC_Response("buzzer", buzzer_on);
}

// Lista de relacao entre as chamadas de RPC e as funcoes que as processam
RPC_Callback callbacks[] = {
  { "setBuzzer",   processSetBuzzer },
};

// Detalhes sobre os sensores e atuadores usados, unidades em que sao enviados e recebidos os valores
DeviceDetails details[] = {
  { "temperature",     "ºC" },
  { "humidity",        "%" },
  { "light_intensity", "lux" },
  { "noise",           "boolean" },
  { "motion",          "boolean" },
  { "tvoc",            "ppb" },
  { "co2",             "ppm" },
  { "buzzer",          "{value: boolean}" },
};

// Semaforos para controlar a concorrencia no envio de mensagens para o ThingsBoard  e nas comunicacoes através de I2C
SemaphoreHandle_t  sema_tb_send; 
SemaphoreHandle_t  sema_i2c; 

// Inicializar o objeto SensorBox passando os dados necessários para conexao a rede Wi-Fi WP2-PSK e ao ThingsBoard
SensorBox sb(WIFI_AP_NAME, WIFI_PASSWORD, THINGSBOARD_SERVER, TOKEN);

// Inicializar o objeto SensorBox passando os dados necessários para conexao a rede Wi-Fi WPA2 Enterprise(experimental) e ao ThingsBoard
// SensorBox sb(WIFI_AP_NAME, "WIFI_USERNAME", WIFI_PASSWORD, THINGSBOARD_SERVER, TOKEN); 

// Inicializar o objeto SensorBox passando os dados necessários para conexao a rede Wi-Fi WP2-PSK e ao ThingsBoard atraves do provisionamento do dispositivo
// SensorBox sb(WIFI_AP_NAME, WIFI_PASSWORD, THINGSBOARD_SERVER, DEVICE_NAME, PROVISION_DEVICE_KEY, PROVISION_DEVICE_SECRET);


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Wire.begin();

  // Configuracao inicial dos LEDs de estado, colocando o LED da ligacao a internet e o da ligacao ao ThingsBoard a vermelho, indicando ainda nao estarem ativas as comunicacoes
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(INTERNET_LED, pixels.Color(50, 0, 0));
  pixels.setPixelColor(THINGSBOARD_LED, pixels.Color(50, 0, 0));
  pixels.show();

  // Inicializar o pino do atuador campainha
  pinMode(BUZZER_PIN, OUTPUT);
  // Inicializar o pino do sensor de ruido
  pinMode(NOISE_SENSOR_PIN, INPUT);
  // Inicializar o pino do sensor de presenca
  pinMode(MOTION_SENSOR_PIN, INPUT);

  // Inicializar comunicacoes I2C com o sensor de temperatura e humidade
  temp_humid_sensor_init();

  // Inicializar comunicacoes I2C com o sensor de luminosidade
  light_meter.begin();

  // Inicializar os semaforos para controlar a concorrencia no envio de mensagens para o ThingsBoard  e nas comunicacoes através de I2C
  sema_tb_send = xSemaphoreCreateMutex();
  sema_i2c = xSemaphoreCreateMutex();

  // Limpeza do token guardado na EEPROM proviniente do provisionamento do dispositivo
  // clearSavedToken();

  // Estabelecer comunicacao com a rede Wi-Fi e o servidor do ThingsBoard
  sb.initComms();
  // Enviar os detalhes dos sensores e atuadores deste dispositivo
  sb.sendDeviceDetails(details, COUNT_OF(details));
  // Subscrever as mensagens das chamadas RPC que acionam os atuadores
  sb.subscribeActuatorsCmds(callbacks, COUNT_OF(callbacks));

  // Subscrever a mensagens relativas a atualizacoes remotas de firmware
  // sb.checkFirmwareUpdates(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);

  // Criacao de tarefas no FreeRTOS para cada sensor, atuador e para os LEDs de estado
  xTaskCreate(
    led_strip_task,    // Function that should be called
    "Led Strip Task",  // Name of the task (for debugging)
    2500,              // Stack size (words)
    NULL,              // Parameter to pass
    1,                 // Task priority
    NULL               // Task handle
  );

  xTaskCreate(
    temperature_humidity_task,            
    "Temperature/Humidity Sensor Task",   
    3000,                                 
    NULL,                                
    1,                                    
    NULL                                 
  );

  xTaskCreate(
    light_sensor_task,   
    "Light Sensor Task",   
    2500,            
    NULL,            
    1,               
    NULL             
  );

  xTaskCreate(
    tvoc_sensor_task,   
    "TVOC/eCO2 Sensor Task",   
    3000,            
    NULL,            
    1,               
    NULL             
  );

  xTaskCreate(
    noise_sensor_task,     
    "Noise Sensor Task",    
    2500,            
    NULL,           
    1,               
    NULL             
  );

  xTaskCreate(
    motion_sensor_task,     
    "Motion Sensor Task",    
    2500,            
    NULL,             
    1,               
    NULL             
  );

  xTaskCreate(
    buzzer_task,     
    "Buzzer Actuator Task",    
    600,             
    NULL,             
    2,                
    NULL            
  );
}

// Ciclo principal, corre sempre que houver tempo livre entre as tarefas do FreeRTOS
void loop() {
  // Manutenção das comunicacoes
  sb.loop();
}

// Verificar se o estados dos LEDs devem ser alterados, caso precisem altera de acordo com a logica implementada
void led_strip_task(void *arg) {

  while (true) {

    if (WiFi.status() != WL_CONNECTED) {  // WiFi connected
      pixels.setPixelColor(INTERNET_LED, pixels.Color(50, 0, 0));
    } else {
      pixels.setPixelColor(INTERNET_LED, pixels.Color(0, 50, 0));
    }

    sb_connected = sb.connected();
  
    if (!sb_connected) {  // thingsboard connected
      pixels.setPixelColor(THINGSBOARD_LED, pixels.Color(50, 0, 0));
    } else {
      pixels.setPixelColor(THINGSBOARD_LED, pixels.Color(0, 50, 0));
    }

    if (sent_temp_humid) { 
      pixels.setPixelColor(TEMP_HUMID_LED, pixels.Color(0, 0, 50));
      sent_temp_humid = false;
    } else {
      pixels.setPixelColor(TEMP_HUMID_LED, pixels.Color(0, 0, 0));
    }

    if (sent_light_intensity) { 
      pixels.setPixelColor(LIGHT_INTENSITY_LED, pixels.Color(0, 0, 50));
      sent_light_intensity = false;
    } else {
      pixels.setPixelColor(LIGHT_INTENSITY_LED, pixels.Color(0, 0, 0));
    }

    if (sent_tvoc_co2) { 
      pixels.setPixelColor(TVOC_CO2_LED, pixels.Color(0, 0, 50));
      sent_tvoc_co2 = false;
    } else {
      pixels.setPixelColor(TVOC_CO2_LED, pixels.Color(0, 0, 0));
    }

    if (sent_noise) { 
      pixels.setPixelColor(NOISE_LED, pixels.Color(0, 0, 50));
      sent_noise = false;
    } else {
      pixels.setPixelColor(NOISE_LED, pixels.Color(0, 0, 0));
    }

    if (sent_motion) { 
      pixels.setPixelColor(MOTION_LED, pixels.Color(0, 0, 50));
      sent_motion = false;
    } else {
      pixels.setPixelColor(MOTION_LED, pixels.Color(0, 0, 0));
    }

    if (buzzer_on) { 
      pixels.setPixelColor(BUZZER_LED, pixels.Color(0, 0, 50));
    } else {
      pixels.setPixelColor(BUZZER_LED, pixels.Color(0, 0, 0));
    }

    // Atualiza a fita de LEDs de acordo com as alteracoes feitas
    pixels.show();

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay(LED_STRIP_TASK_DELAY / portTICK_PERIOD_MS);

  }
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
void temperature_humidity_task(void *arg) {
  float temperature;
  float last_temperature;
  int humidity;
  int last_humidity;
  bool got_sample;

  while (true) { 
    if (sb_connected) {
      if (xSemaphoreTake(sema_i2c, portMAX_DELAY) == pdPASS) {
        got_sample = sht.readSample();
        xSemaphoreGive(sema_i2c);
      }
      if ( got_sample ) {
          
        temperature = sht.getTemperature();
        temperature = roundf(temperature * 10) / 10;
    
        if (isnan(temperature)) {
          Serial.println("Failed to read from temperature sensor!");
        } else {
          // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
          if (abs(last_temperature - temperature) > TEMP_THRESHOLD_SEND || isnan(last_temperature)) {
            // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
            if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
              Serial.print("Sending Temperature...");
              Serial.println(temperature);
              // Envia a temperatura como float
              sb.sendTelemetryFloat("temperature", temperature);
              xSemaphoreGive(sema_tb_send);
  
              sent_temp_humid = true;
              last_temperature = temperature;
            }
          }
        }
      
    
        humidity = sht.getHumidity();
    
        if (isnan(humidity)) {
          Serial.println("Failed to read from humidity sensor!");
        } else {
          // Envia o novo valor apenas se este for diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
          if (last_humidity != humidity || isnan(last_humidity)) {
            // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
            if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
              Serial.print("Sending humidity...");
              Serial.println(humidity);
              // Envia a humidade como int
              sb.sendTelemetryInt("humidity", humidity);
              xSemaphoreGive(sema_tb_send);
  
              sent_temp_humid = true;
              last_humidity = humidity;
            }
          }
        }
        
        
      }
    }

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay( TEMP_HUMID_TASK_DELAY / portTICK_PERIOD_MS);
  }
}


// Fazer a leitura da luminosidade e enviar se necessario
void light_sensor_task(void *arg) {
  float light_intensity;
  float last_light_intensity;

  while (true) {
    if (sb_connected) {
      // Aguarda a sua vez para usar o I2C, a tarefa fica parada ate ter prioridade no semaforo
      if (xSemaphoreTake(sema_i2c, portMAX_DELAY) == pdPASS) {
        light_intensity = light_meter.readLightLevel();
        xSemaphoreGive(sema_i2c);
      }
      light_intensity = roundf(light_intensity * 10) / 10;
  
      if (isnan(light_intensity)) {
        Serial.println("Failed to read from light sensor!");
      } else {
        // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
        if (abs(last_light_intensity - light_intensity) > LIGHT_THRESHOLD_SEND || isnan(last_light_intensity)) {
          // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
          if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
            Serial.print("Sending Light Intensity...");
            Serial.println(light_intensity);
            // Envia a luminosidade como float
            sb.sendTelemetryFloat("light_intensity", light_intensity);
            xSemaphoreGive(sema_tb_send);
  
            sent_light_intensity = true;
            last_light_intensity = light_intensity;
          }
        }
      }
    }

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay( LIGHT_TASK_DELAY / portTICK_PERIOD_MS);
  }
}


// Fazer a leitura da qualidade do ar (TVOC e CO2) e enviar se necessario
void tvoc_sensor_task(void *arg) {
  u16 tvoc, co2, last_sent_tvoc, last_sent_co2;
  s16 err;

  while (sgp_probe() != STATUS_OK) {
      Serial.println("No air quality sensor found!");
      delay(100);
  }

  while (sgp_iaq_init() != STATUS_OK) {
      Serial.println("Error in air quality sensor init!");
      delay(100);
  }

  while (true) {
    if (sb_connected) {
      
      // Aguarda a sua vez para usar o I2C, a tarefa fica parada ate ter prioridade no semaforo
      if (xSemaphoreTake(sema_i2c, portMAX_DELAY) == pdPASS) {
        err = sgp_measure_iaq_blocking_read(&tvoc, &co2);
        xSemaphoreGive(sema_i2c);
      }

      // Se nao houver erro na leitura dos dados do sensor entra
      if (err == STATUS_OK) {
        // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
        if (abs(last_sent_tvoc - tvoc) > TVOC_THRESHOLD_SEND || isnan(last_sent_tvoc)) {
          // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
          if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
            Serial.print("Sending TVOC...");
            Serial.println(tvoc);
            // Envia o valor de TVOC como int
            sb.sendTelemetryInt("tvoc", tvoc);
            xSemaphoreGive(sema_tb_send);
  
            sent_tvoc_co2 = true;
            last_sent_tvoc = tvoc;
          }
        }
        // Envia o novo valor apenas se este for significativamente diferente do ultimo enviado, ou se ainda não tiver enviado nenhum
        if (abs(last_sent_co2 - co2) > CO2_THRESHOLD_SEND || isnan(last_sent_co2)) {
          // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
          if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
            Serial.print("Sending CO2...");
            Serial.println(co2);
            // Envia o valor de CO2 como int
            sb.sendTelemetryInt("co2", co2);
            xSemaphoreGive(sema_tb_send);
  
            sent_tvoc_co2 = true;
            last_sent_co2 = co2;
          }
        }
        
      } else {
        Serial.println("Failed to read from TVOC and eCO2 sensor!");
      }
    }

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay( TVOC_TASK_DELAY / portTICK_PERIOD_MS);
  }
}


// Fazer a leitura do sensor de ruido e enviar se necessario
void noise_sensor_task(void *arg) {
  bool noise;
  bool last_sent_noise;
  unsigned long noise_delay_off_started;

  while (true) {
    if (sb_connected) {
      
      noise = digitalRead(NOISE_SENSOR_PIN); 
  
      if (isnan(noise)) {
        Serial.println("Failed to read from sound sensor!");
      } else {
        // Envia se o estado do ruido for diferente do ultimo enviado e o atraso entre mudanças de estados já tiver sido cumprido, ou se ainda nao tiver enviado nada
        if (last_sent_noise != noise && (millis() - noise_delay_off_started > NOISE_DELAY_OFF ) || isnan(last_sent_noise)) {  // If noise state changed
          // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
          if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
            Serial.print("Sending Noise level...");
            Serial.println(noise);
            // Envia o estado do ruido como boolean
            sb.sendTelemetryBool("noise", noise);
            xSemaphoreGive(sema_tb_send);
  
            sent_noise = true;
            last_sent_noise = noise;
            // Se for detetado ruido reinicia o contador do atraso para sair do estado que diz haver ruido
            if (noise) {  
              noise_delay_off_started = millis();
            }
          }
        // Se for detado ruido inicia o contador que bloqueia este estado durante um tempo predefinido    
        } else if (noise) {
          noise_delay_off_started = millis();
        }
      }
    }

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay( NOISE_TASK_DELAY / portTICK_PERIOD_MS);
  }
}


// Fazer a leitura do sensor de movimento e enviar se necessario
void motion_sensor_task(void *arg) {
  bool motion;
  bool last_sent_motion;

  while (true) {
    if (sb_connected) {
      
      motion = digitalRead(MOTION_SENSOR_PIN); 
  
      if (isnan(motion)) {
        Serial.println("Failed to read from motion sensor!");
      } else {
        // Envia se o estado de presenca for diferente do ultimo enviado, ou se ainda nao tiver enviado nenhum
        if (last_sent_motion != motion || isnan(last_sent_motion)) {
          // Aguarda a sua vez para enviar mensagens ao ThingsBoard, a tarefa fica parada ate ter prioridade no semaforo
          if (xSemaphoreTake(sema_tb_send, portMAX_DELAY) == pdPASS) {
            Serial.print("Sending Motion state...");
            Serial.println(motion);
            // Envia o estado de movimento como boolean
            sb.sendTelemetryBool("motion", motion);
            xSemaphoreGive(sema_tb_send);
  
            sent_motion = true;
            last_sent_motion = motion;
          }
        }
      }
    }

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay( MOTION_TASK_DELAY / portTICK_PERIOD_MS);
  }
}


// Acionar a campainha consoante o estado definido e caso a campainha esteja ativada faz ela tocar de forma intermitente
void buzzer_task(void *arg) {

  while (true) {
    if (sb_connected) {
      if (buzzer_on) {
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
      } else if (digitalRead(BUZZER_PIN)) {
        digitalWrite(BUZZER_PIN, LOW);
      }
    }

    // Pausa a funcao durante um atraso definido, nao bloqueia a execucao do restante codigo
    vTaskDelay(BUZZER_STATE_TIME / portTICK_PERIOD_MS);
  }
}
