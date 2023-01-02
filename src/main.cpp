#include <Arduino.h>

// targets Helteck Wifi Kit 32 V3 
// https://heltec.org/project/wifi-kit-32-v3/
// Schematic: https://resource.heltec.cn/download/WiFi_Kit_32_V3/HTIT-WB32_V3_Schematic_Diagram.pdf


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

#include "battery.h"

WiFiClientSecure wifi_client;
PubSubClient mqtt_client(wifi_client);

Battery battery;


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


// user input button
const int pin_user_button = 0;

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

String device_id;


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

const uint32_t vbat_mv_plugged = 4050;
const uint32_t vbat_mv_unplugged = 3999;
const uint32_t loop_interval_ms = 1000;
const uint32_t data_sample_interval_ms = 5 * 60 * 1000; // 5 minute sample time
const uint32_t light_on_button_press_ms = 5*60*1000; // time to keep display on while on battery power

struct LoopMonitor {
  uint32_t loop_ms = 0;
  uint32_t last_loop_ms = 0;
  uint32_t loop_count = 0;
  void begin_loop() {
    last_loop_ms = loop_ms;
    loop_ms = millis();
    ++loop_count;
  }
  bool every_n_ms(uint32_t interval_ms) {
    return ::every_n_ms(last_loop_ms, loop_ms, interval_ms);
  }

  uint32_t ms_elapsed_since_ms(uint32_t ms) {
    return loop_ms - ms;
  }

};

LoopMonitor loop_monitor;

uint32_t last_user_button_press_millis = 0;
bool user_requested_display_on = true;
bool display_is_on = false;

void user_button_pressed() {
  user_requested_display_on = !display_is_on;
  last_user_button_press_millis = loop_monitor.loop_ms;
}

void setup() {
  Wire.begin(pin_oled_sda,pin_oled_scl);
  Serial.begin(921600);
  delay(5000); // wait for serial monitor
  device_id = String(ESP.getEfuseMac(), HEX);
  Serial.println("device_id: "+ device_id);
  Serial.println("Initializing display");
  
  init_display();
  battery.init();

  // scan_i2c_addresses();
  pinMode(26, INPUT_PULLUP);
  temperature_probe.begin();
  pinMode(26, INPUT_PULLUP);
  temperature_probe.setResolution(12);
  setup_wifi();
  mqtt_client.setServer(mqtt_server_address, 8883);
  maintain_mqtt_connection();

  pinMode(pin_user_button, INPUT);
  attachInterrupt(pin_user_button, user_button_pressed, FALLING);
  Serial.println("End of setup");
}

void loop() {
  static Statistics probe_0_statistics;
  static Statistics probe_1_statistics;
  static float temp_f_0 = NAN;
  static float temp_f_1 = NAN;
  static float mean_temp_f_0 = NAN;
  static float mean_temp_f_1 = NAN;
  

  loop_monitor.begin_loop();

  battery.read_millivolts();
  Serial.print("battery millivolts: "); Serial.println(battery.millivolts);

  bool force_display_on = user_requested_display_on && loop_monitor.ms_elapsed_since_ms(last_user_button_press_millis)<light_on_button_press_ms;

  Serial.print("user_requested_display_on:");
  Serial.println(user_requested_display_on);

  
  if (display_is_on) {
    // turn off display if unplugged and no user requeste
    if ( user_requested_display_on == false 
         || (battery.millivolts <= vbat_mv_unplugged && !force_display_on)) {
      display_is_on = false;
      display.displayOff();
    }
  } else  {
    // turn on display if plugged in or user requested
    if( (user_requested_display_on && battery.millivolts >= vbat_mv_plugged) || force_display_on) {
      init_display();
      display_is_on = true;
      display.displayOn();
    }
  }

  // get temperature
  if(loop_monitor.every_n_ms(1000)) {
    temperature_probe.setWaitForConversion(false);
    temperature_probe.requestTemperatures(); // Send the command to get temperatures
  }


  if(loop_monitor.every_n_ms(1000)) {
      // send to hivemq cloud
      maintain_mqtt_connection();
      if(mqtt_client.connected()) {
        String topic = device_id+"/battery_volts";
        publish_mqtt_value(topic.c_str(), battery.millivolts/1000.0);
      }
  }

  if(loop_monitor.every_n_ms(1000)) {
    temp_f_0 = temperature_probe.getTempFByIndex(0);
    if (temp_f_0 < -100) temp_f_0 = NAN;
    temp_f_1 = temperature_probe.getTempFByIndex(1);
    if (temp_f_1 < -100) temp_f_1 = NAN;
    probe_0_statistics.add_reading(temp_f_0);
    probe_1_statistics.add_reading(temp_f_1);
    if(probe_0_statistics.count >= 5) {
      mean_temp_f_0 = probe_0_statistics.mean();
      probe_0_statistics.reset();

      // send to hivemq cloud
      maintain_mqtt_connection();
      if(mqtt_client.connected()) {
        String topic = device_id+"/temp_0";
        publish_mqtt_value(topic.c_str(), mean_temp_f_0);
      }

    }
    if(probe_1_statistics.count >= 5) {
      mean_temp_f_1 = probe_1_statistics.mean();
      probe_1_statistics.reset();
      maintain_mqtt_connection();
      if(mqtt_client.connected()) {
        String topic = device_id+"/temp_1";
        publish_mqtt_value(topic.c_str(), mean_temp_f_1);
      }
    }
  }
    
  if(display_is_on) {
    display.clear();
    battery.draw_battery(display);
    char buff[80];
    display.setFont(ArialMT_Plain_24);
    if(isnan(mean_temp_f_0)) {
      sprintf(buff, "--");
    }
    else {
      sprintf(buff, "%.1f", mean_temp_f_0);
    }

    display.drawString(0, 0, buff);
    if(isnan(mean_temp_f_1)) {
      sprintf(buff, "--");
    }
    else {
      sprintf(buff, "%.1f", mean_temp_f_1);
    }
    display.drawString(0, 24, buff);
    float graph_min = 65;
    float graph_max = 85;
    const int oled_width = 128;
    const int oled_height = 64;
    const int graph_y = 52;
    display.drawRect(0,graph_y + 1, oled_width * (temp_f_0 - graph_min) / (graph_max-graph_min),1);
    display.drawRect(0,graph_y, oled_width * (mean_temp_f_0 - graph_min) / (graph_max-graph_min),3);

    display.drawRect(0, graph_y+6, oled_width * (temp_f_1 - graph_min) / (graph_max-graph_min),1);
    display.drawRect(0, graph_y+5, oled_width * (mean_temp_f_1 - graph_min) / (graph_max-graph_min),3);

    display.display();
  }

  const int delay_ms = 1;
  vTaskDelay(delay_ms / portTICK_PERIOD_MS);

  
}