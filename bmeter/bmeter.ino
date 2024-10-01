//------------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------------
// source code: https://github.com/fortalbrz/dc_power_meter/
//
// Jorge Albuquerque (2024) - https://linkedin.com/in/jorgealbuquerque
// https://www.jorgealbuquerque.com
//
#define DEBUG_MODE false                            // enables/disables serial debugging messages
#define USE_DISPLAY true                            // enables/disables serial debugging messages
#define USE_MQTT true                               // enables/disables serial debugging messages
#define USE_BLINKING_LED true                       // enables/disables serial debugging messages

//------------------------------------------------------------------------------------------------------------------
//
// Configuration flags (enables or disables features in order to "skip" unwanted hardware)
//
//------------------------------------------------------------------------------------------------------------------
// Wi-fi setup
#define WIFI_SSID "jorge cps"                       // Wi-fi SSID
#define WIFI_PASSWORD "casa1976bonita1980"          // Wi-fi password
// MQTT setup
#define MQTT_BROKER_ADDRESS "192.168.68.10"         // MQTT broker server IP
#define MQTT_BROKER_PORT 1883                       // MQTT broker port
#define MQTT_USERNAME "mqtt-user"                   // can be omitted if not needed
#define MQTT_PASSWORD "mqtt"                        // can be omitted if not needed
// MQTT topics
#define MQTT_COMMAND_TOPIC "bmeter/cmd"             // MQTT topic for sending commands to the meter
#define MQTT_STATUS_TOPIC "bmeter/state"            // MQTT topic for meter status
#define MQTT_AVAILABILITY_TOPIC "bmeter/available"  // MQTT topic for availability notification (home assistant "unavailable" state)
#define MQTT_PUBLISH_DATA_TOPIC "bmeter/data"       // MQTT topic for energy measurements data
#define MQTT_DEVICE_ID "bmeter_12fmo43iowerwe2"     // MQTT session identifier
// others
#define MEASURE_WAIT_TIME 1000                      // sampling rate: time between measurement samples (mileseconds)
#define MEASURE_SAMPLES 300                         // measure time (5 min = 300 seconds)
#define ADC_MAX_VOLTAGE 1.2 * 3.3                   // reference AD converter voltage    
#define ADC_VOLTAGE_THRESHOLDING 1.2 * 0.11         // minimum voltage level               
#define MQTT_STATUS_UPDATE_TIME 120000              // time for send and status update (default: 2 min)
#define MQTT_AVAILABILITY_TIME 60000                // elapsed time to send MQTT availability, in miliseconds (default: 1 min)
#define BUILTIN_LED_BLINKING_TIME 1000              // built-in led  blinking period
#define PUBLISH_OFFSET 3
#define SERIAL_BAUDRATE 9600                        // serial monitor baud rate (only for debuging)
//
// pins definitions (ModeMCU)
//
#define VOLTAGE_SENSOR_PIN A0                       // A0: AD converter
#define PUSH_BUTTON_START_PIN D1                    // D1: pull-up (high) - tactile push button "start"
#define PUSH_BUTTON_RESET_PIN D2                    // D2: pull-up (high) - tactile push button "reset"

#define OLED_SCL_PIN D5                             // Oled display SCL (I2C)
#define OLED_SDA_PIN D6                             // Oled display SDA (I2C) 
//
// MQTT protocol commands
//
#define MQTT_COMMAND_START "start"                  // start energy mensurement (5 min)
#define MQTT_COMMAND_RESET "reset"                  // clears current energy accumulator
#define MQTT_COMMAND_REFRESH "refresh"              // update MQTT status (debug)
#define MQTT_COMMAND_NETINFO "netinfo"              // shows network information
#define MQTT_COMMAND_SAVE "save"                    // shows network information
//
// Measured components (k ohms)
//
#define RESISTOR_100K 96.26
#define RESISTOR_1M_001 1024.0
#define RESISTOR_1M_002 1022.0
#define RESISTOR_1M_003 1005.0
//
// Oled display features
//
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
//
// Display messages
//
#define MSG_STARTING "iniciando..."
#define MSG_PRESS_START "Aperte iniciar!"
#define MSG_CONNECTING_WIFI "conectando wi-fi: "
#define MSG_TIME "tempo: 0"


//------------------------------------------------------------------------------------------------------------------

// librarys (see doc above)
#if (USE_MQTT== true)
  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
#endif
#if (USE_DISPLAY == true)
  #include <Wire.h>
  #include <Adafruit_SSD1306.h>
  #include <U8g2_for_Adafruit_GFX.h>
#endif 

#if (USE_MQTT == true)
  WiFiClient espClient;
  PubSubClient MQTT(espClient);
#endif
#if (USE_DISPLAY == true)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  U8G2_FOR_ADAFRUIT_GFX gfx;
#endif 

// voltage divider
float r1 = RESISTOR_1M_001 + RESISTOR_1M_002 + RESISTOR_1M_003;
float r2 = RESISTOR_100K;
float factor = (r1 + r2) / r2;

// internal states
bool _measuring = false;
float _voltage = 0;
float _power = 0;
float _energy = 0;
float _energyLast = 0;
unsigned int _counter = 0;

bool _blink = false;
unsigned long _lastAvailabilityTime = 0;
unsigned long _lastStatusUpdateTime = 0;
unsigned long _lastBuiltinLedBlinkingTime = 0;
unsigned long _lastButtonPressTime = 0;
float _measures[MEASURE_SAMPLES];

//------------------------------------------------------------------------------------------------------------------
//
// prototypes
//
//------------------------------------------------------------------------------------------------------------------
void readValues();
void displayValues();
ICACHE_RAM_ATTR void onStart();
ICACHE_RAM_ATTR void onReset();
#if (USE_MQTT == true)
  void connectWiFi();
  void connectMqtt();
  void onMqttMessage(char* topic, byte* payload, unsigned int length);
  void updateMqttStates();
  void pubishData();
  void showNetworkInfo();
  String toStr(const char* label, int value, const bool& end);
  String toStr(const char* label, float value, const bool& end);
  String toStr(const char* label, const bool& state, const bool& end);
  String toStr(const char* label, const String& state, const bool& quote, const bool& end);
#endif

//--------------------------------------------------------------------------------------------------
//
// main code
//
//--------------------------------------------------------------------------------------------------
void setup() {
  //
  // initialization
  //
  #if (DEBUG_MODE == true)
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println(F(" "));
    Serial.println(F("-- Debug Mode --"));
  #endif

  // "reset" and "start" push buttons: internal pull-up 
  // (activated by the configuration "INPUT_PULLUP")
  pinMode(PUSH_BUTTON_RESET_PIN, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_START_PIN, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_RESET_PIN), onReset, FALLING);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_START_PIN), onStart, FALLING);
  
  // Iniciar display e configurar interface
  #if (USE_DISPLAY == true)
    Wire.begin(14,12);
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    gfx.begin(display); 

    // clears the graphics buffer
    display.clearDisplay();
    // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
    gfx.setFont(u8g2_font_helvR10_tf);  
    // use u8g2 transparent mode (this is default)
    gfx.setFontMode(1);
    // left to right (this is default)
    gfx.setFontDirection(0);
    // apply Adafruit GFX color
    gfx.setForegroundColor(WHITE);
  #endif

  // wifi connect
  #if (USE_MQTT == true)    
    
    #if (DEBUG_MODE == true)
      Serial.print(F("Connecting to wifi network: "));
      Serial.println(WIFI_SSID);
    #endif
    #if (USE_DISPLAY == true)
      gfx.setCursor(0, 15);
      gfx.print(MSG_CONNECTING_WIFI);
      gfx.setCursor(0, 30);
      gfx.print(WIFI_SSID);
      display.display();
    #endif

    // Wi-fi connection
    connectWiFi();

    // MQTT broker setup  
    MQTT.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
    MQTT.setCallback(onMqttMessage);    
    updateMqttStates();
  #else

    #if (USE_DISPLAY == true)
      gfx.setCursor(0, 15);
      gfx.print(MSG_STARTING);
      display.display();
    #endif

  #endif
}


void loop(void) {
  
  // blinking led
  #if (USE_BLINKING_LED == true)
    unsigned long currentTime = millis();
    if (currentTime - _lastBuiltinLedBlinkingTime > BUILTIN_LED_BLINKING_TIME) {
      digitalWrite(BUILTIN_LED, (_blink ? LOW : HIGH));
      _blink = !_blink;
      _lastBuiltinLedBlinkingTime = currentTime;
    }
  #endif

  // reads values from AD converter
  readValues();

  // shows values
  displayValues();

  if (_measuring){
    // "energy measuring" procedure
    if (_counter < MEASURE_SAMPLES) {
      // store "power" measures on cache
      _measures[_counter] = _power;     

    } else {
      // finishing the energy measurement 
      // (stores last "energy" sample)
      _measuring = false;
      _energyLast = _energy;
      _counter = 0;      
      delay(100);

      #if (USE_MQTT == true)
        // updates MQTT
        connectWiFi();
        connectMqtt();
        updateMqttStates();
        publishData();
      #endif
      return;
    }

    // interval between samples (i.e. sampling rate)
    delay(MEASURE_WAIT_TIME);
    return;    
  }

  #if (USE_MQTT == true)
    // ensures MQTT broker connection
    if (!MQTT.connected())
      connectMqtt();

    // sends MQTT "availability" message
    if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
      MQTT.publish(MQTT_AVAILABILITY_TOPIC, "online");
      _lastAvailabilityTime = millis();
    }
  #endif
      
  #if (USE_MQTT == true)
    // status update (with max timespan for 5 minutes)
    if ((millis() - _lastStatusUpdateTime) > MQTT_STATUS_UPDATE_TIME) {      
      updateMqttStates();    
    }  

    // mqtt loop
    MQTT.loop();

    // keeps wifi connected
    connectWiFi();
  #endif
  
  // waits 
  delay(200);
}

//--------------------------------------------------------------------------------------------------

void readValues() {
  //
  // reads voltage values from AD converter
  //  
  int value = analogRead(A0);
  // ensure range [0, 1023]
  if (value < 0)
    value = 0;
  if (value > 1023)
    value = 1023;

  // current voltage (limited to 100v) [v]
  _voltage = ADC_MAX_VOLTAGE * factor * value / 1023;
  // avoid floating voltage (floor voltage limiar)
  if (_voltage < ADC_VOLTAGE_THRESHOLDING)
    _voltage = 0;

  // current power [w]  
  _power = _voltage * _voltage / r1;

  // energy [J]: power integral (1 second sample)
  _energy += _power;

  // increment samples counter   
  _counter++;
}

//--------------------------------------------------------------------------------------------------

void displayValues() {
  //
  // Show values
  //  
  #if (USE_DISPLAY == true)    
    display.clearDisplay();
    
    String text;
    if (_measuring) {
      // shows time to finish on "measuring" mode
      int remaining = (_counter > MEASURE_SAMPLES) ? 0 : MEASURE_SAMPLES - _counter;
      int min = remaining / 60;
      int sec = remaining % 60; 
      text = String(MSG_TIME) + String(min, DEC) + String(F(":"));
      if (sec < 10)
        text += String(F("0"));
      text += String(sec, DEC);
    } else {      
      if (_energyLast == 0) {
        // shows a "press start" message (if no previous energy measurements)
        text = String(MSG_PRESS_START);
      } else {
        // shows last energy measurements
        text = String(F("E: ")) + String(_energyLast, 6) + String(F(" J"));
      }
    }        
    gfx.setCursor(0,15);
    gfx.print(text.c_str());

    // shows voltage
    text = String(F("V: ")) + String(_voltage, 6) + String(F(" vdc"));
    gfx.setCursor(0,30);
    gfx.print(text.c_str());
    
    // shows power
    text = String(F("P: ")) + String(_power, 6) + String(F(" w"));
    gfx.setCursor(0,45);
    gfx.print(text.c_str());

    // shows energy
    text = String(F("E: ")) + String(_energy, 6) + String(F(" J"));
    gfx.setCursor(0,60);
    gfx.print(text.c_str());

    display.display();
  #endif
  
  #if (DEBUG_MODE == true)
      Serial.print(F("V: "));
      Serial.print(_voltage, 6);
      Serial.print(F(", P: "));
      Serial.print(_power, 6);
      Serial.print(F(", E: "));
      Serial.print(_energy, 6);
      Serial.print(F(", last: "));
      Serial.println(_energyLast, 6);
  #endif
}

//--------------------------------------------------------------------------------------------------

void onReset() {
  //
  // On "reset" push button
  // resets current energy accumulator ISR
  //
  _counter = 0;
  _energy = 0;
  _measuring = false;  
  #if (USE_MQTT == true)
    updateMqttStates();
  #endif
}
 

void onStart() {
  //
  // On "start" push button
  // starts energy measurement ISR
  //
  _counter = 0;
  _energy = 0;
  _measuring = true;
  #if (USE_MQTT == true)
    updateMqttStates();
  #endif
}


//--------------------------------------------------------------------------------------------------
//
// MQTT
//
//--------------------------------------------------------------------------------------------------
#if (USE_MQTT == true)

  void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    //
    // On MQTT message
    //
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';  // NULL;

    #if (DEBUG_MODE == true)
      Serial.print(F(" - mqtt command: ["));
      Serial.print(msg);
      Serial.println(F("]"));
    #endif

    // decode Home Assistant command (MQTT)
    if (strcmp(msg, MQTT_COMMAND_RESET) == 0) {
      // resets current energy accumulator
      onReset();
    } 
    
    // avoid processing during energy measurements
    if (_measuring)    
      return;
    
    if (strcmp(msg, MQTT_COMMAND_START) == 0) {
      // starts new energy measurement
      onStart();

    } else if (strcmp(msg, MQTT_COMMAND_SAVE) == 0) {
      // publish data
      publishData();            

    } else if (strcmp(msg, MQTT_COMMAND_REFRESH) == 0) {
      // updated mqtt states (debug)
      updateMqttStates();

    } else if (strcmp(msg, MQTT_COMMAND_NETINFO) == 0) {
      // shows network information
      #if (USE_DISPLAY == true)
        showNetworkInfo();
        delay(15000);
      #endif
    }
  }

  void updateMqttStates() {
    //
    // MQTT publish states update
    //
    if (MQTT.connected() && !_measuring) {

      String json = String(F("{"));
      json += toStr("voltage", _voltage, false);
      json += toStr("power", _power, false);
      json += toStr("energy", _energy, false);
      json += toStr("last", _energyLast, false);
      json += toStr("rssi", WiFi.RSSI(), false);
      json += toStr("measuring", _measuring, true);

      unsigned int n = json.length() + 1;
      char message[n];
      json.toCharArray(message, n);

      #if (DEBUG_MODE == true)
          Serial.print(F("mqtt update: "));
          Serial.println(message);
      #endif

      MQTT.publish(MQTT_STATUS_TOPIC, message);

      _lastStatusUpdateTime = millis();
    }
  }

  void connectMqtt() {
    //
    // Connects to MQTT broker
    //
    while (!MQTT.connected()) {
      #if (DEBUG_MODE == true)
          Serial.print(F("connecting to MQTT broker: "));
          Serial.println(MQTT_BROKER_ADDRESS);
      #endif

      // retry until connection
      connectWiFi();

      if (MQTT.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        #if (DEBUG_MODE == true)
              Serial.println(F("MQTT broker connected!"));
        #endif        
        // subscribe to command topic on connection
        MQTT.subscribe(MQTT_COMMAND_TOPIC);
        delay(50);
        MQTT.publish(MQTT_AVAILABILITY_TOPIC, "online");
        delay(50);
        updateMqttStates();
        delay(50);
        
      } else {
        #if (DEBUG_MODE == true)
              Serial.println(F("Fail connecting to MQTT broker (retry in 2 secs)."));
        #endif
        delay(2000);
      }
    }
  }


  void connectWiFi() {
    //
    // connects to WiFi
    //
    if (WiFi.status() == WL_CONNECTED)
      return;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // retry until connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      #if (DEBUG_MODE == true)
          Serial.print(F("."));
      #endif
    }

    #if (DEBUG_MODE == true)
      byte mac[6];
      WiFi.macAddress(mac);

      Serial.println();
      Serial.print(F("WiFi connected: "));
      Serial.println(WIFI_SSID);

      Serial.print(F(" - IP address: "));
      Serial.println(WiFi.localIP());

      Serial.print(F(" - MAC: "));
      for (unsigned int i = 5; i >= 0; i--){
        Serial.print(mac[i], HEX);
        if (i == 0)
          Serial.println();
        else
          Serial.print(F(":"));
      }
      
      Serial.print(F(" - RSSI [db]: "));
      Serial.println(WiFi.RSSI());    
    #endif    

    #if (USE_DISPLAY == true)
      showNetworkInfo();
      delay(2000);
    #endif
  }

  void publishData() {
    //
    //
    //
    if (MQTT.connected() && !_measuring) {

      // publish data (point by point)      
      float energy = 0;
      float value = 0;
      for (int i = -1 * PUBLISH_OFFSET; i < (MEASURE_SAMPLES + PUBLISH_OFFSET); i++) {
        value = (i < 0 || i >= MEASURE_SAMPLES)? 0 : _measures[i];
        energy += value;

        String json = String(F("{"));
        json += toStr("time", i + 1 + PUBLISH_OFFSET, false);
        json += toStr("power", value, false);
        json += toStr("energy", energy, true);

        unsigned int n = json.length() + 1;
        char message[n];
        json.toCharArray(message, n);

        MQTT.publish(MQTT_PUBLISH_DATA_TOPIC, message);
        delay(50);
      }
    }
  }

  void showNetworkInfo() {
    //
    // show network information
    //
    #if (USE_DISPLAY == true)
      display.clearDisplay();

      String text = String(F("SSID: ")) + String(WIFI_SSID);
      gfx.setCursor(0, 15);
      gfx.print(text.c_str());

      text = String(F("IP: ")) + WiFi.localIP().toString();
      gfx.setCursor(0, 30);
      gfx.print(text.c_str());

      text = String(F("RSSI: ")) + String(WiFi.RSSI()) + String(F("db"));
      gfx.setCursor(0, 45);
      gfx.print(text.c_str());

      byte mac[6];
      WiFi.macAddress(mac);
      text = String(F("["));
      for (int i = 5; i >= 0; i--) {
         text += String(mac[i], HEX);
         if (i > 0)
           text += String(F(":"));
      } 
      text += String(F("]"));
      gfx.setCursor(0, 60);
      gfx.print(text.c_str());
      
      display.display();
    #endif 
  }

  //--------------------------------------------------------------------------------------------------

  String toStr(const char* label, const String& state, const bool& quote = false , const bool& end = false) {
    //
    // writes a string line with the format:
    //     "label": "on/off",
    //
    String text = String(F("\"[LABEL]\": ")) 
      + (quote ? String(F("\"[VALUE]\"")) : String(F("[VALUE]"))) 
      + (end ? String(F("}")) : String(F(", ")));

    text.replace(F("[LABEL]"), label);
    text.replace(F("[VALUE]"), state);
    return text;
  }
  
  String toStr(const char* label, int value, const bool& end = false) {
    //
    // writes a string line with the format:
    //     "label": 3,
    //  
    return toStr(label, String(value, DEC), false, end);
  }

  String toStr(const char* label, float value, const bool& end = false) {
    //
    // writes a string line with the format:
    //     "label": 1.000000,
    //  
    return toStr(label, String(value, 16), false, end);
  }

  String toStr(const char* label, const bool& state, const bool& end = false) {
    //
    // writes a string line with the format:
    //     "label": "on/off",
    //
    return toStr(label, String(state ? F("on") : F("off")), true, end);
  }

#endif

//--------------------------------------------------------------------------------------------------