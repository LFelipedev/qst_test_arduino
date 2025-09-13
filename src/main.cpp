#include <Arduino.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 

#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "semphr.h"

const int potenciometro = A0;
const int set_button = 13;

TaskHandle_t potenciometroTaskH;
TaskHandle_t temperaturaTaskH;

uint16_t valor_potenciometro; 
uint8_t temp_pot;
uint8_t temp_set;

SemaphoreHandle_t SerialMutex;
LiquidCrystal_I2C lcd(0x3F, 20, 4);

void sendGantt(const char *name, unsigned int stime, unsigned int etime);
void potenciometroTask(void *arg);
void temperaturaTask(void *arg);

void setup() {
  Serial.begin(9600);
  
  lcd.init(); 
  lcd.backlight(); 
  lcd.clear(); 

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
      temp_pot = map(valor_potenciometro, 0, 1023, 25, 42);

      if(abs(temp_pot - temp_pot_ant) >= 1) {
        temp_pot_ant = temp_pot;
        prev_temp_set = true;
        lcd.setCursor(0,0);
        lcd.print("TEMPERATURA DESEJADA");
        lcd.setCursor(0,1);
        lcd.print(temp_pot);
        lcd.setCursor(2,1);
        lcd.print("C");
        Serial.print("Teperatura desejada: ");
        Serial.println(temp_pot); 
      }

      button_state = !digitalRead(set_button);

      if(button_state && !prev_button_state && prev_temp_set) {
        temp_set = temp_pot;
        prev_temp_set = false;
        lcd.setCursor(0,2);
        lcd.print("TEMPERATURA SETADA");
        lcd.setCursor(0,3);
        lcd.print(temp_set);
        lcd.setCursor(2,3);
        lcd.print("C");
        Serial.print("Temperatura setada: ");
        Serial.println(temp_set);       
      }

      prev_button_state = button_state;

      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void temperaturaTask(void *arg) {

    while (true) {

      
    Serial.println("TEMP");

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}