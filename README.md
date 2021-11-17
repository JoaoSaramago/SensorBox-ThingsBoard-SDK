# SensorBox-ThingsBoard SDK

Biblioteca para microcontroladores que simplifica a conexão e comunicação destes com a plataforma de IoT ThingsBoard. Esta extende a biblioteca oficial já existente para Arduino que comunica com o servidor através da API de MQTT. Agiliza o uso das seguintes funcionalidades:

- Conexão à rede Wi-Fi, incluindo rede Eduroam (experimental)
- Conexão ao servidor
- Manutenção das conexões
- Envio de detalhes da SensorBox
- Subscrição a comandos para os atuadores
- Provisionamento de dispositivos
- Atualizações OTA

A biblioteca é compatível com o Arduino Uno e a sua placa Wi-Fi(baseada no ESP8266), o ESP8266 e o ESP32. Com esta biblioteca o mesmo ficheiro de código pode ser usado nos três microcontroladores mencionados.
 
