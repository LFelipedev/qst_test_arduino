#include <Arduino.h>
#include <Display.h>

Display oled_128x64;

void setup() {
    
    oled_128x64.setTargetTemperature(25); 
    oled_128x64.setTemperature(20);
}

void loop() {
    
}
