#include <Arduino.h>

// targets Helteck Wifi Kit 32 V3 
// https://heltec.org/project/wifi-kit-32-v3/


// for the OLED Display
#include "SSD1306Wire.h"

// for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>


// MQTT stuff, tutorial at
// https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
// https://community.hivemq.com/t/hivemq-using-esp32-and-nodered/1291
#include "passwords.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

WiFiClientSecure wifi_client;
PubSubClient mqtt_client(wifi_client);


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  wifi_client.setCACert(root_ca);
}

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
  uint8_t device_count=0;
  for(uint8_t i=0;i<0x80;i++){
    Wire.beginTransmission(i);
    uint8_t ec=Wire.endTransmission(true); // if device exists on bus, it will aCK
    if(ec==0){ // Device ACK'd
      if(i<16)Serial.print('0');
      Serial.print(i,HEX);
      ++device_count;
    }
    else Serial.print(".."); // no one answered
    Serial.print(' ');
    if ((i&0x0f)==0x0f)Serial.println();
    }
  Serial.print(" I2C Scan found ");
  Serial.print(device_count);
  Serial.println(" devices");
}


void maintain_mqtt_connection() {
  if(!mqtt_client.connected()) {
    if(mqtt_client.connect("tank1", mqtt_username, mqtt_password)) {
      Serial.println("connected to mqtt with username and password");
    } else {
      Serial.println("failed to connect to mqtt with username and password");
    }
  }

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
  setup_wifi();
  mqtt_client.setServer(mqtt_server_address, 8883);
  maintain_mqtt_connection();
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

void publish_mqtt_value(const char * topic, float x) {
      char value_string[8]; 
      const int  digits_after_decimal = 2;
      const int min_width = 1;
      dtostrf(x, min_width, digits_after_decimal, value_string);

      if(mqtt_client.publish(topic, value_string)) {
        Serial.print("succesfully sent ");
        Serial.print(value_string);
        Serial.print(" to ");
        Serial.print(topic);
        Serial.println();
      } else {
        Serial.print("failed sending a message to ");
        Serial.print(topic);
        Serial.println();
      }  
}

void loop() {
  return;
  static unsigned long loop_ms = 0;
  static unsigned long last_loop_ms = 0;
  static int64_t loop_count = 0;
  static Statistics probe_0_statistics;
  static Statistics probe_1_statistics;
  last_loop_ms = loop_ms;
  loop_ms = millis();
  ++loop_count;
  static float mean_temp_f_0 = NAN;
  static float mean_temp_f_1 = NAN;

  // 170ma without delay -> 106ma (battery off)
  // 80ma with delay 500ms
  // 80ma with delay 10 ms
  // 45 ma with esp_light_sleep_start 10ms
  // 8ma with 45 ma spikes from siglent with esp_light_sleep
  // 5ma with 45 ma spikes from siglent with esp_light_sleep and only displaying "."
  const int delay_ms = 1000;
  //vTaskDelay(delay_ms / portTICK_PERIOD_MS);
  delay(100);
  //esp_sleep_enable_timer_wakeup(delay_ms * 1000);
  //esp_light_sleep_start();

  // get temperature
  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    temperature_probe.setWaitForConversion(false);
    temperature_probe.requestTemperatures(); // Send the command to get temperatures
  }


  if(every_n_ms(last_loop_ms, loop_ms, 1000)) {
    float temp_f_0 = temperature_probe.getTempFByIndex(0);
    float temp_f_1 = temperature_probe.getTempFByIndex(1);
    probe_0_statistics.add_reading(temp_f_0);
    probe_1_statistics.add_reading(temp_f_1);
    if(probe_0_statistics.count >= 5) {
      mean_temp_f_0 = probe_0_statistics.mean();
      probe_0_statistics.reset();

      // send to hivemq cloud
      maintain_mqtt_connection();
      if(mqtt_client.connected()) {
        publish_mqtt_value("esp32/temp_0", mean_temp_f_0);
      }

    }
    if(probe_1_statistics.count >= 5) {
      mean_temp_f_1 = probe_1_statistics.mean();
      probe_1_statistics.reset();
      maintain_mqtt_connection();
      if(mqtt_client.connected()) {
        publish_mqtt_value("esp32/temp_1", mean_temp_f_1);
      }
    }
    
    display.clear();
    char buff[80];
    sprintf(buff, "loop_count - %d", loop_count);
    display.drawString(0, 0, buff);
    sprintf(buff, "latest F: %.1f %.1f", temp_f_0, temp_f_1);
    display.drawString(0, 10, buff);
    sprintf(buff, "smooth F: %.1f %.1f", mean_temp_f_0, mean_temp_f_1);
    display.drawString(0, 20, buff);
    float graph_min = 65;
    float graph_max = 85;
    const int oled_width = 128;
    const int oled_height = 64;
    display.drawRect(0,34, oled_width * (temp_f_0 - graph_min) / (graph_max-graph_min),1);
    display.drawRect(0,33, oled_width * (mean_temp_f_0 - graph_min) / (graph_max-graph_min),3);

    display.drawRect(0,39, oled_width * (temp_f_1 - graph_min) / (graph_max-graph_min),1);
    display.drawRect(0,38, oled_width * (mean_temp_f_1 - graph_min) / (graph_max-graph_min),3);

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
          Serial.print(address[j],HEX);
        }
      }
    }

    Serial.println();

  }
  
}