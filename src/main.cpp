#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define potenciometro A3
#define set_button 13

#define lm35 A0
#define peltier 3

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

void setup()
{
    Serial.begin(9600);

    lcd.init();
    lcd.backlight();
    lcd.clear();

    analogReference(INTERNAL); // usa referência interna de 1,1V
    
    pinMode(set_button, INPUT);
    pinMode(lm35, INPUT);
    pinMode(peltier, OUTPUT);

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

void loop()
{
}

void sendGantt(const char *name, unsigned int stime, unsigned int etime)
{
    if (xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {
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

    while (true)
    {

        valor_potenciometro = analogRead(potenciometro);
        temp_pot = map(valor_potenciometro, 0, 1023, 25, 42);

        if (abs(temp_pot - temp_pot_ant) >= 1)
        {
            temp_pot_ant = temp_pot;
            prev_temp_set = true;
            lcd.setCursor(0, 0);
            lcd.print("TEMPERATURA DESEJADA");
            lcd.setCursor(0, 1);
            lcd.print(temp_pot);
            lcd.setCursor(2, 1);
            lcd.print("C");
            Serial.print("Teperatura desejada: ");
            Serial.println(temp_pot);
        }

        button_state = !digitalRead(set_button);

        if (button_state && !prev_button_state && prev_temp_set)
        {
            temp_set = temp_pot;
            prev_temp_set = false;
            lcd.setCursor(0, 2);
            lcd.print("TEMPERATURA SETADA");
            lcd.setCursor(0, 3);
            lcd.print(temp_set);
            lcd.setCursor(2, 3);
            lcd.print("C");
            Serial.print("Temperatura setada: ");
            Serial.println(temp_set);
        }

        prev_button_state = button_state;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

int set = 40;
int pwm = 0;

double pid;

double error = 0, last_error = 0;
double last_temperature;
long last_process;

double kp = 10, ki = 1.0, kd = 1.5;
double p = 0, i = 0, d = 0;

void temperaturaTask(void *arg) {

    while (true)
    {

        int leitura = analogRead(lm35);
        float tensao = leitura * (1.1 / 1023.0); // referência interna
        int temperature = tensao * 100.0;        // 10 mV/°C

        error = temp_set - temperature;
        float delta = (millis() - last_process) / 1000.0;
        last_process = millis();

        p = error * kp;
        i += (error * ki) * delta;
        d = ((error - last_error) / delta) * kd;

        pid = p + i + d;

        pwm = constrain(pid, 0, 255);

        analogWrite(peltier, pwm);

        Serial.println(temperature);
        last_error = error;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


