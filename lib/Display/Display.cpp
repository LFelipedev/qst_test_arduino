#include "Display.h"

Display::Display() {

    Wire.begin();
    display_oled_128x64.begin();
    display_oled_128x64.clearDisplay();
    display_oled_128x64.setCursor(0, 0);
    display_oled_128x64.setTextColor(WHITE);
    display_oled_128x64.setTextSize(2);
    display_oled_128x64.display();
}

void Display::showDisplay() {

    display_oled_128x64.clearDisplay();
    display_oled_128x64.print("Temperatura setada: ");
    display_oled_128x64.println(this->set_temperature);
    display_oled_128x64.println("Temperatura atual: ");
    display_oled_128x64.print("Temperatura desejada: ");
    display_oled_128x64.print(this->target_temperature);
    display_oled_128x64.display();
}

void Display::setTargetTemperature(uint8_t target) {

    this->target_temperature = target;
    showDisplay();
}

void Display::setTemperature(uint8_t set) {
    
    this->set_temperature = set;
    showDisplay();
}
