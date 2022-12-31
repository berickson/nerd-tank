// battery interface for Heltec ESP32 WiFi Kit V3

#include <Arduino.h>

// for the OLED Display
#include "SSD1306Wire.h"

class Battery {

  const int pin_adc_ctrl = 37;
  const int pin_battery_input = 1;
public:
    float analog_read_average = 0.0;
    uint16_t millivolts = 0.0;

    void init() {
      pinMode(pin_battery_input, ANALOG);

      // voltage divider uses vbat -> 390k -> pin_input-> 100k -> gnd
      // max voltage for 4.25v input should be less than 0.87v
      analogSetPinAttenuation(pin_battery_input, ADC_0db);

      // setting ADC_Ctrl to low sends battery to GPIO 1 to be read
      pinMode(pin_adc_ctrl ,OUTPUT);
    }

    void read_millivolts() {
      uint32_t analog_read_sum = 0;
      const int millivolts_sample_count=100;
      for(int i = 0; i < millivolts_sample_count; ++i) {
        analog_read_sum += analogRead(pin_battery_input);
        delay(1);
      }
      this->analog_read_average = (float)analog_read_sum / millivolts_sample_count;
      this->millivolts = analog_read_average * 3705 / 3262;
    }

    // draws battery image, percentage and voltage to display
    void draw_battery(SSD1306Wire & display) {
      auto mv_min = 3300;
      auto mv_max = 4200;
      display.setColor(BLACK);
      display.fillRect(99,0,29,24);
      display.setColor(WHITE);
      display.drawRect(104,0,12,6);
      display.fillRect(116,2,1,2);

      long percent = constrain(map(this->millivolts,mv_min,mv_max,0,100),0,100);
      uint8_t bars = round(percent / 10.0);
      display.fillRect(105,1,bars,4);
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(127,5,String(percent)+"%");
      display.drawString(127,14,String(round(this->millivolts/1.0)/1000.0,2)+"V");
      display.setTextAlignment(TEXT_ALIGN_LEFT);
    }
};
