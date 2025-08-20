#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h> 

class Display {

    public:
        Display();
        void setTargetTemperature(uint8_t target);
        void setTemperature(uint8_t set);
        
    private:
        Adafruit_SSD1306 display_oled_128x64;
        uint8_t target_temperature;
        uint8_t set_temperature;
        void showDisplay();
};

#endif