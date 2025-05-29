#include <Arduino.h>

#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <OneWire.h>
#include <DallasTemperature.h>

const int sensor_temperatura = 12; 
const int potenciometro = A0;

TaskHandle_t potenciometroTaskH;
TaskHandle_t temperaturaTaskH;

const int send_gantt = 1;
uint16_t valor_potenciometro; 
float temperatura_c;

SemaphoreHandle_t SerialMutex;

OneWire oneWire(sensor_temperatura); 
DallasTemperature sensor(&oneWire); 
DeviceAddress endereco_temp;

void sendGantt(const char *name, unsigned int stime, unsigned int etime);
void potenciometroTask(void *arg);
void temperaturaTask(void *arg);

void setup() {
  Serial.begin(9600); 
  sensor.begin();

  SerialMutex = xSemaphoreCreateMutex();
  
  xTaskCreate(temperaturaTask,            
              "temperaturaTask",          
              256,                
              NULL,               
              1,                  
              &temperaturaTaskH);
  
  xTaskCreate(potenciometroTask,
              "potenciometroTask",
              128,
              NULL,
              0,
              &potenciometroTaskH);
}
  
void loop() {
}

void sendGantt(const char *name, unsigned int stime, unsigned int etime) {
    if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  
        Serial.print("\t\t");
        Serial.print(name);
        Serial.print(": ");
        Serial.print(stime);
        Serial.print(", ");
        Serial.println(etime);
        xSemaphoreGive(SerialMutex);                           
    }
}

void potenciometroTask(void *arg) {

  while(true) {
    valor_potenciometro = analogRead(potenciometro);
    Serial.print("Potenciometro: ");
    Serial.println(valor_potenciometro);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void temperaturaTask(void *arg) {

    while (true) {

      sensor.requestTemperatures();

      if(sensor.getAddress(endereco_temp, 0)) {
        temperatura_c = sensor.getTempC(endereco_temp);
        Serial.print("Temperatura: ");
        Serial.println(temperatura_c);
      } 
      
      else {
        Serial.println("Erro ao tentar ler o sensor");
      }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}