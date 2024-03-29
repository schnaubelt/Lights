#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <HTTPUpdateServer.h>
#include <LittleFS.h>
#include <NTC_Thermistor.h>
#include <SmoothThermistor.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

#define light_name "Dimmable LED Floodlight"  // default light name
#define LIGHT_VERSION 2.1
#define LIGHTS_COUNT 1

// define pins
#define LED_PWM_PIN 23
#define FAN_PWM_PIN 15
#define THERMISTOR_PIN 34

// Needs to be around 8% at least
#define LED_MIN_DUTY 180  // 250

// We want to have 1.5A max, which equals to roughly 85%
#define LED_MAX_DUTY 2500
//#define LED_MAX_DUTY 1000

#define LEDC_CHANNEL_LED 0
#define LEDC_CHANNEL_FAN 1

// use 12 bit precission for LEDC timer, 10 bit for fan
#define LEDC_TIMER_12_BIT 12
#define LEDC_TIMER_10_BIT 10

// use 2000 Hz as a LEDC base frequency
#define LED_PWM_FREQ 2000

// use 25000 Hz as a LEDC base frequency
#define FAN_PWM_FREQ 25000  // todo

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)

//#define USE_STATIC_IP //! uncomment to enable Static IP Adress
#ifdef USE_STATIC_IP
IPAddress strip_ip(192, 168, 0, 95);   // choose an unique IP Adress
IPAddress gateway_ip(192, 168, 0, 1);  // Router IP
IPAddress subnet_mask(255, 255, 255, 0);
#endif

uint8_t scene;
bool light_state[LIGHTS_COUNT], in_transition;
int transitiontime[LIGHTS_COUNT], bri[LIGHTS_COUNT];
float step_level[LIGHTS_COUNT], current_bri[LIGHTS_COUNT];
byte mac[6];

WebServer server(80);
HTTPUpdateServer httpUpdateServer;
WiFiManager wm;

Thermistor* thermistor;
double led_temp;
uint32_t led_duty;
uint32_t fan_duty;

void ledcWriteMappedLimits(uint8_t channel, uint32_t value) {
  // TODO : For now also set FAN_PWM using this value
  ledcWrite(1, value);

  if (value == 0) {
    led_duty = 0;
    ledcWrite(channel, led_duty);

  } else {
    led_duty =
        (value - 1) * (LED_MAX_DUTY - LED_MIN_DUTY) / (4095) + LED_MIN_DUTY;

    ledcWrite(channel, led_duty);
  }
}

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWriteMappedLimits(channel, duty);
}

void blinkLed(uint8_t count, uint16_t interval = 500) {
  for (uint8_t i = 0; i < count; i++) {
    ledcWriteMappedLimits(LEDC_CHANNEL_LED, 0);
    delay(interval);
    ledcWriteMappedLimits(LEDC_CHANNEL_LED, 50);
    delay(2 * interval);
  }
}

void factoryReset() {
  LittleFS.format();
  WiFi.disconnect(false, true);
  blinkLed(5, 500);
  ESP.restart();
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void apply_scene(uint8_t new_scene, uint8_t light) {
  if (new_scene == 0) {
    bri[light] = 144;
  } else if (new_scene == 1) {
    bri[light] = 254;
  } else if (new_scene == 2) {
    bri[light] = 1;
  }
}

void process_lightdata(uint8_t light, float transitiontime) {
  transitiontime *= 33;
  if (light_state[light]) {
    step_level[light] = (bri[light] - current_bri[light]) / transitiontime;
  } else {
    step_level[light] = current_bri[light] / transitiontime;
  }
}

void lightEngine() {
  for (int i = 0; i < LIGHTS_COUNT; i++) {
    if (light_state[i]) {
      if (bri[i] != current_bri[i]) {
        in_transition = true;
        current_bri[i] += step_level[i];
        if ((step_level[i] > 0.0 && current_bri[i] > bri[i]) ||
            (step_level[i] < 0.0 && current_bri[i] < bri[i])) {
          current_bri[i] = bri[i];
        }
        ledcWriteMappedLimits(LEDC_CHANNEL_LED, (int)(current_bri[i] * 16));
      }
    } else {
      if (current_bri[i] != 0) {
        in_transition = true;
        current_bri[i] -= step_level[i];
        if (current_bri[i] < 0) {
          current_bri[i] = 0;
        }
        ledcWriteMappedLimits(LEDC_CHANNEL_LED, (int)(current_bri[i] * 16));
      }
    }
  }
  if (in_transition) {
    delay(2);
    in_transition = false;
  }
}

double readLedTemperature() {
  const double celsius = thermistor->readCelsius();
  return celsius;
}

void fanTempControl(double temp) {
  if (temp > 40.0)
    fan_duty = 4095;
  else if (temp > 35)
    fan_duty = 3000;
  else if (temp > 30)
    fan_duty = 2000;
  else
    fan_duty = 500;

  ledcWrite(LEDC_CHANNEL_FAN, fan_duty);
}

void setup() {
  EEPROM.begin(512);
  ledcSetup(LEDC_CHANNEL_LED, LED_PWM_FREQ, LEDC_TIMER_12_BIT);
  ledcSetup(LEDC_CHANNEL_FAN, FAN_PWM_FREQ, LEDC_TIMER_12_BIT);

  ledcAttachPin(LED_PWM_PIN, LEDC_CHANNEL_LED);
  ledcAttachPin(FAN_PWM_PIN, LEDC_CHANNEL_FAN);

  blinkLed(5);
  ledcWriteMappedLimits(LEDC_CHANNEL_LED, 0);

  Thermistor* originThermistor =
      new NTC_Thermistor(THERMISTOR_PIN, 100000.0, 100000.0, 25, 3950, 4095);

  thermistor = new SmoothThermistor(originThermistor, 20);

  Serial.begin(115200);

#ifdef USE_STATIC_IP
  WiFi.config(strip_ip, gateway_ip, subnet_mask);
#endif

  for (uint8_t light = 0; light < LIGHTS_COUNT; light++) {
    apply_scene(EEPROM.read(2), light);
    step_level[light] = bri[light] / 150.0;
  }

  if (EEPROM.read(1) == 1 || (EEPROM.read(1) == 0 && EEPROM.read(0) == 1)) {
    for (int i = 0; i < LIGHTS_COUNT; i++) {
      light_state[i] = true;
    }
    for (int j = 0; j < 200; j++) {
      lightEngine();
    }
  }
  WiFi.mode(WIFI_STA);
  wm.setDebugOutput(true);
  wm.setConfigPortalTimeout(120);

  bool res;
  res = wm.autoConnect(light_name);

  if (!res) {
    ESP.restart();
  }

  if (!light_state[0]) {
    // Show that we are connected
    ledcWriteMappedLimits(LEDC_CHANNEL_LED, 50);
    delay(500);
    ledcWriteMappedLimits(LEDC_CHANNEL_LED, 0);
  }

  WiFi.macAddress(mac);

  httpUpdateServer.setup(&server);  // start http server

  server.on("/state", HTTP_PUT,
            []() {  // HTTP PUT request used to set a new light state
              DynamicJsonDocument root(1024);
              DeserializationError error =
                  deserializeJson(root, server.arg("plain"));

              if (error) {
                server.send(404, "text/plain", "FAIL. " + server.arg("plain"));
              } else {
                for (JsonPair state : root.as<JsonObject>()) {
                  const char* key = state.key().c_str();
                  int light = atoi(key) - 1;
                  JsonObject values = state.value();
                  int transitiontime = 4;

                  if (values.containsKey("on")) {
                    if (values["on"]) {
                      light_state[light] = true;
                      if (EEPROM.read(1) == 0 && EEPROM.read(0) == 0) {
                        EEPROM.write(0, 1);
                      }
                    } else {
                      light_state[light] = false;
                      if (EEPROM.read(1) == 0 && EEPROM.read(0) == 1) {
                        EEPROM.write(0, 0);
                      }
                    }
                  }

                  if (values.containsKey("bri")) {
                    bri[light] = values["bri"];
                  }

                  if (values.containsKey("bri_inc")) {
                    bri[light] += (int)values["bri_inc"];
                    if (bri[light] > 255)
                      bri[light] = 255;
                    else if (bri[light] < 1)
                      bri[light] = 1;
                  }

                  if (values.containsKey("transitiontime")) {
                    transitiontime = values["transitiontime"];
                  }
                  process_lightdata(light, transitiontime);
                }
                String output;
                serializeJson(root, output);
                server.send(200, "text/plain", output);
              }
            });

  server.on("/state", HTTP_GET,
            []() {  // HTTP GET request used to fetch current light state
              uint8_t light = server.arg("light").toInt() - 1;
              DynamicJsonDocument root(1024);
              root["on"] = light_state[light];
              root["bri"] = bri[light];
              String output;
              serializeJson(root, output);
              server.send(200, "text/plain", output);
            });

  server.on("/detect",
            []() {  // HTTP GET request used to discover the light type
              char macString[32] = {0};
              sprintf(macString, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
                      mac[1], mac[2], mac[3], mac[4], mac[5]);
              DynamicJsonDocument root(1024);
              root["name"] = light_name;
              root["lights"] = LIGHTS_COUNT;
              root["protocol"] = "native_multi";
              root["modelid"] = "LWB010";
              root["type"] = "dimmable_light";
              root["mac"] = String(macString);
              root["version"] = LIGHT_VERSION;
              String output;
              serializeJson(root, output);
              server.send(200, "text/plain", output);
            });

  server.on("/", []() {
    float transitiontime = 4;
    if (server.hasArg("startup")) {
      if (EEPROM.read(1) != server.arg("startup").toInt()) {
        EEPROM.write(1, server.arg("startup").toInt());
        EEPROM.commit();
      }
    }

    for (int light = 0; light < LIGHTS_COUNT; light++) {
      if (server.hasArg("scene")) {
        if (server.arg("bri") == "" && server.arg("hue") == "" &&
            server.arg("ct") == "" && server.arg("sat") == "") {
          if (EEPROM.read(2) != server.arg("scene").toInt()) {
            EEPROM.write(2, server.arg("scene").toInt());
            EEPROM.commit();
          }
          apply_scene(server.arg("scene").toInt(), light);
        } else {
          if (server.arg("bri") != "") {
            bri[light] = server.arg("bri").toInt();
          }
        }
      } else if (server.hasArg("on")) {
        if (server.arg("on") == "true") {
          light_state[light] = true;
          {
            if (EEPROM.read(1) == 0 && EEPROM.read(0) == 0) {
              EEPROM.write(0, 1);
            }
          }
        } else {
          light_state[light] = false;
          if (EEPROM.read(1) == 0 && EEPROM.read(0) == 1) {
            EEPROM.write(0, 0);
          }
        }
        EEPROM.commit();
      } else if (server.hasArg("alert")) {
        if (light_state[light]) {
          current_bri[light] = 0;
        } else {
          current_bri[light] = 255;
        }
      }
      if (light_state[light]) {
        step_level[light] =
            ((float)bri[light] - current_bri[light]) / transitiontime;
      } else {
        step_level[light] = current_bri[light] / transitiontime;
      }
    }
    if (server.hasArg("reset")) {
      ESP.restart();
    }

    String http_content = "<!doctype html>";
    http_content += "<html>";
    http_content += "<head>";
    http_content += "<meta charset=\"utf-8\">";
    http_content +=
        "<meta name=\"viewport\" content=\"width=device-width, "
        "initial-scale=1.0\">";
    http_content += "<title>Light Setup</title>";
    http_content +=
        "<link rel=\"stylesheet\" "
        "href=\"https://unpkg.com/purecss@0.6.2/build/pure-min.css\">";
    http_content += "</head>";
    http_content += "<body>";
    http_content += "<fieldset>";
    http_content += "<h3>Light Setup</h3>";
    http_content +=
        "<form class=\"pure-form pure-form-aligned\" action=\"/\" "
        "method=\"post\">";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"power\"><strong>Power</strong></label>";
    http_content += "<a class=\"pure-button";
    if (light_state[0]) http_content += "  pure-button-primary";
    http_content += "\" href=\"/?on=true\">ON</a>";
    http_content += "<a class=\"pure-button";
    if (!light_state[0]) http_content += "  pure-button-primary";
    http_content += "\" href=\"/?on=false\">OFF</a>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"startup\">Startup</label>";
    http_content +=
        "<select onchange=\"this.form.submit()\" id=\"startup\" "
        "name=\"startup\">";
    http_content += "<option ";
    if (EEPROM.read(1) == 0) http_content += "selected=\"selected\"";
    http_content += " value=\"0\">Last state</option>";
    http_content += "<option ";
    if (EEPROM.read(1) == 1) http_content += "selected=\"selected\"";
    http_content += " value=\"1\">On</option>";
    http_content += "<option ";
    if (EEPROM.read(1) == 2) http_content += "selected=\"selected\"";
    http_content += " value=\"2\">Off</option>";
    http_content += "</select>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"scene\">Scene</label>";
    http_content +=
        "<select onchange = \"this.form.submit()\" id=\"scene\" "
        "name=\"scene\">";
    http_content += "<option ";
    if (EEPROM.read(2) == 0) http_content += "selected=\"selected\"";
    http_content += " value=\"0\">Relax</option>";
    http_content += "<option ";
    if (EEPROM.read(2) == 1) http_content += "selected=\"selected\"";
    http_content += " value=\"1\">Bright</option>";
    http_content += "<option ";
    if (EEPROM.read(2) == 2) http_content += "selected=\"selected\"";
    http_content += " value=\"2\">Nightly</option>";
    http_content += "</select>";
    http_content += "</div>";
    http_content += "<br>";

    // LED STATE

    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"state\"><strong>LED State</strong></label>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"bri\">Brightness</label>";
    http_content +=
        "<input id=\"bri\" name=\"bri\" type=\"range\" min=\"0\" "
        "max=\"100\" value=\"" +
        (String)bri[0] + "\"  \">";
    http_content += "</div>";

    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"led_temp\">Temperature</label>";
    http_content += "<label for=\"bri\"> " + (String)led_temp + " °C</label>";
    http_content += "</div>";

    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"led_duty\">Duty Cycle</label>";
    http_content += "<label for=\"bri\"> " + (String)led_duty + "</label>";
    http_content += "</div>";
    http_content += "<br>";

    // FAN STATE

    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"state\"><strong>Fan State</strong></label>";
    http_content += "</div>";
    http_content += "<div class=\"pure-control-group\">";

    http_content += "<div class=\"pure-control-group\">";
    http_content += "<label for=\"led_duty\">Duty Cycle</label>";
    http_content += "<label for=\"bri\"> " + (String)fan_duty + "</label>";
    http_content += "</div>";

    http_content += "<div class=\"pure-controls\">";
    http_content +=
        "<span class=\"pure-form-message\"><a href=\"/?alert=1\">alert</a> or "
        "<a href=\"/?reset=1\">reset</a></span>";
    http_content += "<label for=\"cb\" class=\"pure-checkbox\">";
    http_content += "</label>";
    http_content +=
        "<button type=\"submit\" class=\"pure-button "
        "pure-button-primary\">Save</button>";
    http_content += "</div>";
    http_content += "</fieldset>";
    http_content += "</form>";
    http_content += "</body>";
    http_content += "</html>";

    server.send(200, "text/html", http_content);
  });

  server.on("/reset", []() {  // trigger manual reset
    server.send(200, "text/html", "reset");
    delay(1000);
    ESP.restart();
  });

  server.on("/factory", []() {  // trigger manual reset
    server.send(200, "text/html", "factory reset");
    factoryReset();
  });

  server.onNotFound(handleNotFound);

  server.begin();
  ledcWriteMappedLimits(LEDC_CHANNEL_LED, 0);

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();

  server.handleClient();
  lightEngine();
  led_temp = readLedTemperature();
  Serial.println(led_temp);
  fanTempControl(led_temp);
}