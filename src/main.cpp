#include <Arduino.h>

// for the OLED Display
#include "SSD1306Wire.h"

// for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// oled display
const int oled_address=0x3c;
const int pin_oled_sda = 17;
const int pin_oled_scl = 18;
const int pin_oled_rst = 21;

// temperature sensor pin
const int pin_one_wire_bus = 26;
OneWire one_wire(pin_one_wire_bus);
DallasTemperature temperature_probe(&one_wire);

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
  pinMode(26, INPUT_PULLUP);
  temperature_probe.begin();
  pinMode(26, INPUT_PULLUP);
  temperature_probe.setResolution(12);
  
}


bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

class Statistics {
  public:
      int count = 0;
      float sum_x = 0.0;
      float sum_x2 = 0.0;



    void reset() {
      count = 0;
      sum_x = 0.0;
      sum_x2 = 0.0;
    }

    void add_reading(float x) {
      count += 1;
      sum_x += x;
      sum_x2 += x*x;
    }

    float mean() {
      return sum_x / count;
    }
};

void loop() {
  static unsigned long loop_ms = 0;
  static unsigned long last_loop_ms = 0;
  static int64_t loop_count = 0;
  static Statistics probe_0_statistics;
  last_loop_ms = loop_ms;
  loop_ms = millis();
  ++loop_count;
  static float mean_temp_f = NAN;


  // get temperature
  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    temperature_probe.setWaitForConversion(false);
    temperature_probe.requestTemperatures(); // Send the command to get temperatures
  }


  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    float temp_f = temperature_probe.getTempFByIndex(0);
    probe_0_statistics.add_reading(temp_f);
    if(probe_0_statistics.count >= 30) {
      mean_temp_f = probe_0_statistics.mean();
      Serial.print("Probe 0 mean: ");
      Serial.println(mean_temp_f);
      probe_0_statistics.reset();
    }


    display.clear();
    char buff[80];
    sprintf(buff, "loop_count - %d", loop_count);
    display.drawString(0, 0, buff);
    sprintf(buff, "degrees F: %.1f", mean_temp_f);
    display.drawString(0, 10, buff);
    display.display();


  }
  if(every_n_ms(last_loop_ms, loop_ms, 10000)) {

    Serial.print("Loops: ");
    Serial.print(loop_count);

    auto temp_probe_count = temperature_probe.getDS18Count();
    Serial.print(" df18 count: " );
    Serial.print(temp_probe_count);

    Serial.print(" temp_f: ");

    for(auto i=0;i<temp_probe_count; ++i) {
      Serial.print(" ");
      Serial.print(temperature_probe.getTempFByIndex(i));
    }


    Serial.print(" addresses: ");
    for(auto i=0;i<temp_probe_count; ++i) {
      DeviceAddress address = {0,0,0,0,0,0,0,0};
      if(temperature_probe.getAddress(address,i)) {
        Serial.print(" ");
        for(auto j=0; j<8; ++j) {
          Serial.print(" ");
          Serial.print(address[j],HEX);
        }
      }
    }

    Serial.println();

  }
  
}