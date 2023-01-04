#include <Arduino.h>
#include <chrono>

int64_t get_system_millis() {
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  auto chrono_millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  return chrono_millis;
}

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
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(100/portTICK_PERIOD_MS);
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

// board LED
const int pin_led = 35;

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


bool every_n_ms(int64_t last_loop_ms, int64_t loop_ms, int64_t ms) {
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
const uint32_t display_hold_ms = 60*1000; // time to keep display on while on battery power

struct LoopMonitor {
  uint64_t loop_ms = 0;
  uint64_t last_loop_ms = 0;
  uint64_t loop_count = 0;


  // init should be called on first power on if using RTC memory
  void init() {
    loop_ms = 0;
    last_loop_ms = 0;
    loop_count = 0;
  }

  void begin_loop() {
    last_loop_ms = loop_ms;
    
    loop_ms = get_system_millis();
    ++loop_count;
  }
  bool every_n_ms(uint64_t interval_ms) {
    return ::every_n_ms(last_loop_ms, loop_ms, interval_ms);
  }

  uint64_t ms_elapsed_since_ms(uint64_t ms) {
    return loop_ms - ms;
  }
};

RTC_NOINIT_ATTR LoopMonitor loop_monitor;
RTC_NOINIT_ATTR uint64_t display_request_millis = 0;
RTC_NOINIT_ATTR bool user_requested_display_on = true;
bool display_is_on = false;
TaskHandle_t wifi_task_handle = nullptr;

void user_button_pressed() {

  auto ms = get_system_millis();

  // "debounce", don't allow more than once per second.
  if(ms - display_request_millis < 1000) {
    return;
  }

  if(digitalRead(pin_user_button) == LOW) {
    user_requested_display_on = !display_is_on;
    display_request_millis = ms;
  }
}

void setup() {
  Wire.begin(pin_oled_sda,pin_oled_scl);
  Serial.begin(921600);
  display_is_on = false;
  Serial.println();

  switch(esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT0:
      user_requested_display_on = true;
      display_request_millis = get_system_millis();
      Serial.println("waking from EXT0");
      // pinMode(pin_led, OUTPUT);
      // digitalWrite(pin_led, HIGH);
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("waking from timer");
      break;

    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      Serial.println("waking from undefined / other");
      loop_monitor.init();
      user_requested_display_on = true;
      break;
  }
  
  device_id = String(ESP.getEfuseMac(), HEX);
  Serial.println("setup for device_id: "+ device_id);
  
  battery.init();

  // scan_i2c_addresses();
  pinMode(26, INPUT_PULLUP);
  temperature_probe.begin();
  pinMode(26, INPUT_PULLUP);
  temperature_probe.setResolution(12);

  pinMode(pin_user_button, INPUT);
  attachInterrupt(pin_user_button, user_button_pressed, FALLING);
}



void upload_data_to_server() {
      if(WiFi.status() != WL_CONNECTED) {
        setup_wifi();
      }
      mqtt_client.setServer(mqtt_server_address, 8883);
      // send to hivemq cloud
      maintain_mqtt_connection();
      if(mqtt_client.connected()) {
        String topic = device_id+"/battery_volts";
        publish_mqtt_value(topic.c_str(), battery.millivolts/1000.0);

        topic = device_id+"/temp_0";
        float temp_f_0 = temperature_probe.getTempFByIndex(0);
        if (temp_f_0 < -100) temp_f_0 = NAN;
        publish_mqtt_value(topic.c_str(), temp_f_0);

        topic = device_id+"/temp_1";
        float temp_f_1 = temperature_probe.getTempFByIndex(1);
        if (temp_f_1 < -100) temp_f_1 = NAN;
        publish_mqtt_value(topic.c_str(), temp_f_1);

      }
}

void wifi_task(void * params) {
  Serial.print("wifi_task: Executing on core ");
  Serial.println(xPortGetCoreID());

  upload_data_to_server();

  Serial.println("wifi_task done ");
  wifi_task_handle = nullptr; 
  vTaskDelete(NULL); // delete this task
}



void loop() {
  static Statistics probe_0_statistics;
  static Statistics probe_1_statistics;
  static float temp_f_0 = NAN;
  static float temp_f_1 = NAN;
  static float mean_temp_f_0 = NAN;
  static float mean_temp_f_1 = NAN;
  static uint64_t wifi_start_ms = 0;
  

  loop_monitor.begin_loop();
  battery.read_millivolts();

    // get temperature
  if(loop_monitor.every_n_ms(1000)) {
    temperature_probe.setWaitForConversion(true);
    temperature_probe.requestTemperatures(); // Send the command to get temperatures
  }

  mean_temp_f_0 = temp_f_0 = temperature_probe.getTempFByIndex(0);
  mean_temp_f_1 = temp_f_1 = temperature_probe.getTempFByIndex(1);
  Serial.printf("loop ms: %ju battery mv: %d temps: %f, %f\n", loop_monitor.loop_ms, battery.millivolts, temp_f_0, temp_f_1);

  bool force_display_on = user_requested_display_on && (loop_monitor.loop_ms - display_request_millis  <display_hold_ms);

  
  if (display_is_on) {
    // turn off display if unplugged and no user requeste
    if ( user_requested_display_on == false 
         || ( (battery.millivolts <= vbat_mv_unplugged) && !force_display_on)) {
      display_is_on = false;
      display.displayOff();
    }
  } else  {
    // turn on display if plugged in or user requested
      if (force_display_on) {
      init_display();
      display_is_on = true;
      display.displayOn();
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

  // kill wifi task if it runs too long
  uint64_t wifi_timeout_ms = 20000;
  if(wifi_task_handle != nullptr && loop_monitor.loop_ms - wifi_start_ms > wifi_timeout_ms) {
    vTaskDelete(wifi_task_handle);
    wifi_task_handle = nullptr;
    Serial.println("WiFi task timed out and was killed");

  }

  if(loop_monitor.every_n_ms(5 * 60 * 1000)) {
    const char * task_name = "wifi_task";
    const uint32_t stack_depth = 10000; // 1000 caused stack overflow
    void * parameters = nullptr;
    u32_t priority = 1; // lower numbers are lower priority
    BaseType_t core_id = 0;

    auto rv = xTaskCreatePinnedToCore(
      wifi_task,
      task_name,
      stack_depth,
      parameters,
      priority,
      &wifi_task_handle,
      core_id);
    if(rv==pdPASS) {
      Serial.println("wifi task created ok");
    } else {
      Serial.print("wifi task create failed with code ");
      Serial.print(rv);
    }
    wifi_start_ms = loop_monitor.loop_ms;
  }
 
  // sleep or delay depending on what is going on
  bool can_light_sleep = (wifi_task_handle == nullptr)
                          && digitalRead(pin_user_button)==HIGH;
  bool can_deep_sleep =  can_light_sleep  
                         && !display_is_on;
  if(can_deep_sleep) {
      const uint32_t sleep_seconds = 10;

      detachInterrupt(pin_user_button);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW);
      esp_deep_sleep(sleep_seconds * 1E6);
  } else {
    if(can_light_sleep) {
      const uint32_t light_sleep_seconds = 5;
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW);
      esp_sleep_enable_timer_wakeup(light_sleep_seconds * 1000 * 1000);
      esp_light_sleep_start();
    } else {
      const int delay_ms = 1000;
      vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
  }
 
}