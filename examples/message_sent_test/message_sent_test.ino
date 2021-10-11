#include <SensorBoxTB.h>
#include <math.h>

#define SERIAL_DEBUG_BAUD    115200

#define WIFI_AP_NAME        "wi-fi_ap_name"
#define WIFI_PASSWORD       "wi-fi_password"

#define TOKEN               "ARDUINO_UNO_TOKEN"
#define THINGSBOARD_SERVER  "thingsboard.rnl.tecnico.ulisboa.pt" 

SensorBox sb(WIFI_AP_NAME, WIFI_PASSWORD, THINGSBOARD_SERVER, TOKEN);


int counter;

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);

  sb.initComms();
  
  for (counter = 0; counter < 100; counter++) {
    light_sensor_task();
    sb.loop();
    delay(700);
  }
  output_test();
}


void loop() { 
}

float calculateSD(unsigned int data[]) {
    float sum = 0.0, mean, SD = 0.0;
    int i;
    for (i = 0; i < 10; ++i) {
        sum += data[i];
    }
    mean = sum / 10;
    for (i = 0; i < 10; ++i) {
        SD += pow(data[i] - mean, 2);
    }
    return sqrt(SD / 10);
}

void swap(unsigned int *p,unsigned int *q) {
   int t;
   
   t=*p; 
   *p=*q; 
   *q=t;
}

void sort(unsigned int a[],unsigned int n) { 
   int i,j,temp;

   for(i = 0;i < n-1;i++) {
      for(j = 0;j < n-i-1;j++) {
         if(a[j] > a[j+1])
            swap(&a[j],&a[j+1]);
      }
   }
}

unsigned int lst[100];
unsigned long start;
unsigned long finish;

void light_sensor_task() { 
  int light_intensity = 1000;

  if (isnan(light_intensity)) {
    Serial.println(F("Failed to get light intensity value!"));
  } else {
    if (true) {
        start = millis();
        sb.sendTelemetryInt("light_intensity", light_intensity);
        finish = millis();
        lst[counter] = finish - start;
        Serial.print(F("#")); Serial.print(counter); Serial.print(F(" Elapsed time: ")); Serial.println(lst[counter]);
    }
  }
}

void output_test() {
  unsigned int sum = 0;
  unsigned int biggest = 0;
  unsigned int smallest = 10000;
  for (int i = 0; i < counter; ++i) {
    sum += lst[i];
    if (lst[i] > biggest) biggest = lst[i];
    if (lst[i] < smallest) smallest = lst[i];
  }
  float avg = sum * 1.0 / counter;
  sort(lst, counter);
  unsigned int median = lst[(counter+1) / 2 - 1];
  Serial.print(F("Mean: ")); Serial.println(avg);
  Serial.print(F("Standard Deviation: ")); Serial.println(calculateSD(lst));
  Serial.print(F("Min: ")); Serial.println(smallest);
  Serial.print(F("Max: ")); Serial.println(biggest);
  Serial.print(F("Median: ")); Serial.println(median);
}
