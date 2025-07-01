#include <Arduino.h>

#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <OneWire.h>
#include <DallasTemperature.h>

const int sensor_temperatura = 12; 
const int potenciometro = A0;
const int set_button = 13;

TaskHandle_t potenciometroTaskH;
TaskHandle_t temperaturaTaskH;

const int send_gantt = 1;
uint16_t valor_potenciometro; 
float temperatura_c;
uint8_t temp_pot;
uint8_t temp_set;

SemaphoreHandle_t SerialMutex;

OneWire oneWire(sensor_temperatura); 
DallasTemperature sensor(&oneWire); 
DeviceAddress endereco_temp;

void sendGantt(const char *name, unsigned int stime, unsigned int etime);
void potenciometroTask(void *arg);
void temperaturaTask(void *arg);

void setup() {
  Serial.begin(115200); 
  sensor.begin();

  pinMode(set_button, INPUT);

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

uint8_t temp_pot_ant = 0;

void potenciometroTask(void *arg) {

  boolean button_state, prev_button_state, prev_temp_set = false;

    while(true) {
      
      valor_potenciometro = analogRead(potenciometro);
      temp_pot = map(valor_potenciometro, 0, 1023, 25, 50);

      if(abs(temp_pot - temp_pot_ant) >= 1) {
        temp_pot_ant = temp_pot;
        prev_temp_set = true;
        Serial.print("Teperatura desejada: ");
        Serial.println(temp_pot); 
      }

      button_state = !digitalRead(set_button);

      if(button_state && !prev_button_state && prev_temp_set) {
        temp_set = temp_pot;
        prev_temp_set = false;
        Serial.print("Temperatura setada: ");
        Serial.println(temp_set);       
      }

      prev_button_state = button_state;

      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void temperaturaTask(void *arg) {

    while (true) {

      sensor.requestTemperatures();

      if(sensor.getAddress(endereco_temp, 0)) {
        temperatura_c = sensor.getTempC(endereco_temp);
        //Serial.print("Temperatura: ");
        //Serial.println(temperatura_c);
      } 
      
      else {
        //Serial.println("Erro ao tentar ler o sensor");
      }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}