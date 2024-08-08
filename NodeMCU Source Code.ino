const int ultrasonicDetectCount = 3;
const int ultrasonicDetectCm = 8;
const unsigned long tazerTimeoutMs = 8000;
const unsigned long dropTimeoutMs = 3000;
const unsigned long firebaseIntervalMs = 2000;
const int lowBattery = 20;

// --------------------------------------- OLED_NODEMCU
#include <Wire.h>
#include "SSD1306Wire.h"
class Oled_Nodemcu {
  private:
    SSD1306Wire display; 
  public:
    Oled_Nodemcu(): display(0x3c, 14, 12) {}

    void setup() {
      display.init();
      display.flipScreenVertically();
    }

    void print(String text, OLEDDISPLAY_TEXT_ALIGNMENT alignment = TEXT_ALIGN_LEFT) {
      display.clear();
      display.setTextAlignment(alignment);
      display.setFont(ArialMT_Plain_24);
      display.drawString(0, 20, text);
      display.display();
    }
};


// ---------------------------------------  ULTRASONIC SENSOR
class Ultrasonic {
  private:
    byte trigPin, echoPin;
    void (*callback)();
    int detectedCm;
    int detectedCount;
    int numDetected = 0;

  public:
    Ultrasonic(byte trigPin, byte echoPin, int detectedCm, int detectedCount = 2) : 
      trigPin(trigPin), echoPin(echoPin), detectedCm(detectedCm), detectedCount(detectedCount) {}

    void setup() {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    int read() {
      digitalWrite(trigPin, LOW);
      delay(2);
      digitalWrite(trigPin, HIGH);
      delay(10);
      digitalWrite(trigPin, LOW);

      long duration = pulseIn(echoPin, HIGH);
      int distance = duration * 0.034 / 2;
      return distance;
    }

    bool detected() {
      if (read() < detectedCm && read() != 0) {
        if (numDetected <= detectedCount) {
          numDetected++;
        }
      } else {
        numDetected = 0;
      }

      return numDetected > detectedCount;
    }
};


// --------------------------------------- SET TIMEOUT
class SetTimeout {
  private:
    unsigned long delayTime;   // Delay time in milliseconds
    unsigned long lastMillis;  // Last time checked
    void (*callback)();        // Function pointer to be called

  public:
    bool activated;            // Flag to indicate if setTimeout is active
    SetTimeout() {}

    void start(unsigned long delayTime, void (*callback)()) {
      this->delayTime = delayTime;
      this->callback = callback;
      this->lastMillis = millis();
      this->activated = true;
    }

    void loop() {
      if (this->activated) {
        unsigned long currentMillis = millis();
        if (currentMillis - this->lastMillis >= this->delayTime) {
          this->callback();
          this->activated = false;
        }
      }
    }
};


// --------------------------------------- Relay
class Relay {
  private:
    byte pin;
    bool activeLow;

  public:
    Relay(byte pin, bool activeLow = true) {
      this->pin = pin;
      this->activeLow = activeLow;
    }

    void setup() {
      pinMode(pin, OUTPUT);
      off();
    }

    void on() {
      digitalWrite(pin, !activeLow);
    }

    void off() {
      digitalWrite(pin, activeLow);
    }

    void onIf(bool isOn) {
      if (isOn) {
        this->on();
      } else {
        this->off();
      }
    }
};


// --------------------------------------- LED
class LED {
  private:
    byte pin;
    bool activeLow;

  public:
    LED(byte pin, bool activeLow = false) {
      this->pin = pin;
      this->activeLow = activeLow;
    }

    void setup() {
      pinMode(pin, OUTPUT);
      off();
    }

    void on() {
      digitalWrite(pin, !activeLow);
    }

    void off() {
      digitalWrite(pin, activeLow);
    }

    void onIf(bool isOn) {
      if (isOn) {
        this->on();
      } else {
        this->off();
      }
    }
};


// --------------------------------------- VOLTAGE READER
class VoltageReader {
  private:
    byte pin;
    int minRead;
    int maxRead;
    int factor;
  
  public:
    VoltageReader(byte pin, int minRead = 406, int maxRead = 482, int factor = 10)
    {
      this->pin = pin;
      this->minRead = minRead;
      this->maxRead = maxRead;
      this->factor = factor;
    }

    int read() {
      return analogRead(pin);
    }

    int computeBatteryLevel() {
      int vout = read();

      if (vout < minRead) return 0;
      if (vout > maxRead) return 100;
     return (int)(floor((float)(vout-minRead)/(maxRead-minRead) * 100/factor) * factor);
    }
};



// --------------------------------------- FIREBASE
  #include <Firebase_ESP_Client.h>
  #include <addons/TokenHelper.h>
  #if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
    #include <WiFi.h>
  #elif defined(ESP8266)
    #include <ESP8266WiFi.h>
  #endif
  #define WIFI_SSID "Giga2"
  #define WIFI_PASSWORD "Dignidad.14321"
  #define API_KEY "AIzaSyBp2lKbdacejdX5MPzBMSd4rz5K4Ki1Suw"
  #define FIREBASE_PROJECT_ID "pest-controller"
  #define USER_EMAIL "admin@gmail.com"
  #define USER_PASSWORD "admin123"
class FirebaseHelper {
  private:
    FirebaseData fbdo;
    FirebaseAuth auth;
    FirebaseConfig config;

  public:
    void setup() {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      Serial.print("Connecting to Wi-Fi");
      unsigned long ms = millis();
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
      }
      Serial.println();
      Serial.print("Connected with IP: ");
      Serial.println(WiFi.localIP());
      Serial.println();

      Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

      /* Assign the api key (required) */
      config.api_key = API_KEY;

      /* Assign the user sign in credentials */
      auth.user.email = USER_EMAIL;
      auth.user.password = USER_PASSWORD;

      /* Assign the callback function for the long running token generation task */
      config.token_status_callback = tokenStatusCallback;  // see addons/TokenHelper.h

      #if defined(ESP8266)
            // In ESP8266 required for BearSSL rx/tx buffer for large data handle, increase Rx size as needed.
          fbdo.setBSSLBufferSize(2048 /* Rx buffer size in bytes from 512 - 16384 */,
                                2048 /* Tx buffer size in bytes from 512 - 16384 */);
      #endif

      // Limit the size of response payload to be collected in FirebaseData
      fbdo.setResponseSize(2048);

      Firebase.begin(&config, &auth);

      Firebase.reconnectWiFi(true);
    }

    void write(const char *documentPath, FirebaseJson content, const char *fields) {
      if (!Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath, content.raw(), fields)) {
        Serial.println(fbdo.errorReason());
      }
    }

    FirebaseJson read(const char *documentPath, const char *fields) {
      FirebaseJson readJson;
      if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath, fields)) {
        readJson.setJsonData(fbdo.payload());
      } else {
        Serial.println(fbdo.errorReason());
      }

      return readJson;
    }
};


// --------------------------------------- SET INTERVAL
class SetInterval {
  private:
    unsigned long interval;   // Interval time in milliseconds
    unsigned long lastMillis; // Last time the function was called
    void (*callback)();       // Function pointer to be called

  public:
    SetInterval() {}

    void setup(unsigned long interval, void (*callback)()) {
      this->interval = interval;
      this->callback = callback;
      this->lastMillis = millis();
    }

    void loop() {
      unsigned long currentMillis = millis();
      if (currentMillis - this->lastMillis >= this->interval) {
        this->callback();
        this->lastMillis = currentMillis;
      }
    }
};


// --------------------------------------- CONNECTING TO ARDUINO UART
// (Arduino -> NodeMCU) (D5 -> D6) (D4 -> D7)
#include <SoftwareSerial.h>
typedef void (*StringArrayFunction)(String[], int);
class SerialNodeMCU {
  private:
    byte rxPin;
    byte txPin;
    EspSoftwareSerial::UART myPort;

    static int splitString(String input, char delimiter, String* resultArray, int maxItems) {
      int splitCount = 0;
      int startIndex = 0;
      int endIndex = input.indexOf(delimiter);

      while (endIndex >= 0 && splitCount < maxItems) {
        resultArray[splitCount] = input.substring(startIndex, endIndex);
        resultArray[splitCount].trim();
        startIndex = endIndex + 1;
        endIndex = input.indexOf(delimiter, startIndex);
        splitCount++;
      }

      // Add the last part of the input (or the whole input if there are fewer than maxItems parts)
      if (splitCount < maxItems) {
        resultArray[splitCount] = input.substring(startIndex);
        resultArray[splitCount].trim();
        splitCount++;
      }

      return splitCount;
    }

  public:
  
    SerialNodeMCU(byte rxPin, byte txPin) {
      this->rxPin = rxPin;
      this->txPin = txPin;
    }

    void setup() {
      myPort.begin(9600, SWSERIAL_8N1, rxPin, txPin, false);
      if (!myPort) { // If the object did not initialize, then its configuration is invalid
        Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
        while (1) { // Don't continue with invalid configuration
          delay (1000);
        }
      } 
    }


    void on_receive(int receive_size, StringArrayFunction callback) {
      if (myPort.available() > 0) {
        String receivedData = myPort.readStringUntil('\n');
        // Serial.println("Received Data: " + receivedData);

        // Split the received data using the comma as the delimiter
        String values[receive_size];
        int numValues = splitString(receivedData, ',', values, receive_size);

        if (numValues == receive_size) {
          // Assuming the format is "float,float,String,float"
          callback(values, receive_size);
        }
      }
    }

    void print(String str) {
      myPort.print(str);
    }

    void println(String line) {
      myPort.println(line);
    }
};


// --------------------------------------- MAIN STATE
enum State {
  IDLING,
  TAZING,
  DROPING,
};


// --------------------------------------------------------------- CLASSES
Ultrasonic ultrasonic(D1, D2, ultrasonicDetectCm, ultrasonicDetectCount);
State state = IDLING;
SetTimeout tazerSetTimeout;
SetTimeout dropSetTimeout;
Relay relay(D0, true);
LED redLed(D3);
LED greenLed(D4);
LED orangeLed(D8);
FirebaseHelper firebase;
SerialNodeMCU nano(D6, D7);
SetInterval firebaseInterval;
VoltageReader voltageReader(A0, 750, 1024, 10); // pin, minRead, maxRead, factor
// VoltageReader voltageReader(A0, 869, 1024, 10); // pin, minRead, maxRead, factor
Oled_Nodemcu oled;

// --------------------------------------------------------------- VARIABLES
bool alert = true;
bool on;
int max_bin = -1;
int total_rats_bag = -1;
int total_rats_killed = -1;
int battery = 100;

// --------------------------------------------------------------- FUNCTIONS
void taze() {
  state = TAZING;
  relay.on();
  tazerSetTimeout.start(tazerTimeoutMs, drop);
}

void drop() {
  state = DROPING;
  relay.off();

  nano.print("exterminated,");
  nano.println(String(total_rats_bag + 1));
  Serial.print("Sent to nano: exterminated: ");
  Serial.println(total_rats_bag + 1);

  // switch (total_rats_bag) {
  //   case 1:
  //   case 5:
  //   case 10:
  //   case 15:
      
  //     break;
    
  //   default:
  //     break;
  // }
  firebase_write_taze();

  
  dropSetTimeout.start(2 * dropTimeoutMs, idle);
}

void idle() {
  state = IDLING;
}

void stateLoop() {
  switch(state) {
    case IDLING:
      Serial.print("Idling... ultrasonic: ");
      Serial.print(ultrasonic.read());
      Serial.print("\tvoltage: ");
      Serial.print(ultrasonic.read());
      Serial.print("\tbattery: ");
      Serial.print(voltageReader.computeBatteryLevel());
      Serial.print("\ton: ");
      Serial.println(on);
      if (ultrasonic.detected() && on) {
        taze();
      }
      break;

    case TAZING:
      Serial.println("Tazing...");
      break;

    case DROPING:
      Serial.println("Dropping...");
      break;

    default:
      break;
  }
}


void firebase_read() {
  FirebaseJson readJson = firebase.read("device/readings", "total_rats_bag,total_rats_killed,max_bin");

  FirebaseJsonData firebaseJsonData;
  readJson.get(firebaseJsonData, "fields/total_rats_bag/integerValue");
  total_rats_bag = firebaseJsonData.to<int>();
  readJson.get(firebaseJsonData, "fields/total_rats_killed/integerValue");
  total_rats_killed = firebaseJsonData.to<int>();
  readJson.get(firebaseJsonData, "fields/max_bin/integerValue");
  max_bin = firebaseJsonData.to<int>();

  on = total_rats_bag < max_bin;
  
  Serial.print("\ttotal_rats_bag: ");
  Serial.print(total_rats_bag);
  Serial.print("\ttotal_rats_killed: ");
  Serial.print(total_rats_killed);
  Serial.print("\tmax_bin: ");
  Serial.print(max_bin);
  Serial.println();
  firebase_write_battery();
}


unsigned long lastLowBattery = millis();
bool initialSend = false;

void firebase_write_taze() {
  total_rats_bag = -1;
  total_rats_killed = -1;
  max_bin = -1;
  
  


  firebase_read();

  if (total_rats_bag < 0 || total_rats_killed < 0 || max_bin < 0) {
    return;
  }

  FirebaseJson content;
  content.set("fields/alert/booleanValue", alert);
  content.set("fields/total_rats_bag/integerValue", total_rats_bag + 1);
  content.set("fields/total_rats_killed/integerValue", total_rats_killed + 1);

  firebase.write("device/readings", content, "alert,total_rats_bag,total_rats_killed");
}

void firebase_write_battery() {
  FirebaseJson content;
  content.set("fields/battery/integerValue", battery);
  firebase.write("device/readings", content, "battery");
}


// --------------------------------------------------------------- SETUP
void setup() {
  Serial.begin(9600);
  greenLed.setup();
  redLed.setup();
  orangeLed.setup();
  greenLed.on();
  redLed.off();
  orangeLed.off();
  firebase.setup();
  ultrasonic.setup();
  relay.setup();
  nano.setup();
  firebaseInterval.setup(firebaseIntervalMs, firebase_read);
  firebase_read();
  oled.setup();
  greenLed.off();

}

// --------------------------------------------------------------- LOOP
void loop() {
  // nano.println("ext");
  // oled.print(String("    ") + String(voltageReader.read())+String("%"));
  tazerSetTimeout.loop();
  dropSetTimeout.loop();
  stateLoop();
  redLed.onIf(!on);
  orangeLed.onIf(on && state == TAZING);
  firebaseInterval.loop();

  battery = voltageReader.computeBatteryLevel();

  if (battery <= lowBattery && (!initialSend || millis() - lastLowBattery > 60000 * 5)) {
    lastLowBattery = millis();
    nano.print("low,");
    nano.println(String(battery));
    initialSend = true;
  }
}


// OTHER USE CASES
void other_use_cases() {
  voltageReader.read();
}
