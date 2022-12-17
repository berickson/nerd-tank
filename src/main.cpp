#include <Arduino.h>
#include "SSD1306Wire.h"


// oled display
const int oled_address=0x3c;
const int pin_oled_sda = 17;
const int pin_oled_scl = 18;
const int pin_oled_rst = 21;

SSD1306Wire display(oled_address, pin_oled_sda, pin_oled_scl);

void init_display() {
  MAIN_ESP32_HAL_GPIO_H_
  // prepare oled display
  pinMode(pin_oled_rst, OUTPUT);
  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);
  display.init();
}



void scan_i2c_addresses(){
  Serial.println("\n Scanning I2C Addresses");
  uint8_t cnt=0;
  for(uint8_t i=0;i<0x7F;i++){
    Wire.beginTransmission(i);
    uint8_t ec=Wire.endTransmission(true); // if device exists on bus, it will aCK
    if(ec==0){ // Device ACK'd
      if(i<16)Serial.print('0');
      Serial.print(i,HEX);
      cnt++;
    }
    else Serial.print(".."); // no one answered
    Serial.print(' ');
    if ((i&0x0f)==0x0f)Serial.println();
    }
  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");
}


void setup() {
  Wire.begin(pin_oled_sda,pin_oled_scl);
  Serial.begin(921600);
  delay(5000); // wait for serial monitor
  Serial.println("Initializing display");
  
  init_display();
  Serial.println("End of setup");
  scan_i2c_addresses();
  
}


bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

void loop() {
  static unsigned long loop_ms = 0;
  static unsigned long last_loop_ms = 0;
  static int64_t loop_count = 0;
  last_loop_ms = loop_ms;
  loop_ms = millis();
  ++loop_count;


  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    Serial.print("Loops: ");
    Serial.print(loop_count);
    Serial.println();

    display.clear();
    char buff[80];
    sprintf(buff, "loop_count - %d", loop_count);
    display.drawString(0, 0, buff);
    display.display();
  }
  
}