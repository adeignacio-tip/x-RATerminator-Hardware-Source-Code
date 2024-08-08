
const char *phone = "+639xxxxxxxxx";
const unsigned long dropTimeoutMs = 1500;
const unsigned long servo_rotate_angle = 300;

// --------------------------------------- MYSERVO
#include <Servo.h>
class MyServo {
  private:
    byte pin;
    Servo servo;
    enum State {IDLE, FORWARD, REVERSE};
    State state = State::IDLE;
    unsigned long startTimestamp;
    int from;
    int to;
    unsigned long duration;
    bool withReverse = false;

  public:

    MyServo(byte pin): pin(pin) {}

    void setup() {
      servo.attach(pin);
      rotate(servo_rotate_angle, servo_rotate_angle, 1000);
    }

    void loop() {
      Serial.print("Servo state: ");
      Serial.println(state);
      // IDLE STATE
      if (state == State::IDLE) {
        servo.write(servo_rotate_angle);
        return;
      }

      unsigned long timeElapsed = millis() - startTimestamp;

      // FORWARD STATE
      if (state == State::FORWARD) {
        if (timeElapsed > duration) {
          startTimestamp = millis();
          state = State::REVERSE;
          return;
        }
        
        long angle = map(timeElapsed, 0, duration, from, to);
        servo.write(angle);
        return;
      }

      // REVERSE STATE
      if (state == State::REVERSE) {
        if (timeElapsed > duration) {
          state = State::IDLE;
          // servo.detach();
          return;
        }
        
        long angle = map(timeElapsed, 0, duration, to, from);
        servo.write(angle);
        return;
      }
    }

    void rotate(int from, int to, unsigned long duration, bool withReverse = false) {
      if (state != State::IDLE) return;
      
      this->state = State::FORWARD;
      this->startTimestamp = millis();
      this->from = from;
      this->to = to;
      this->duration = duration;
      this->withReverse = withReverse;
      // servo.attach(pin);
    }
};


// --------------------------------------- SIM900
#include <SoftwareSerial.h>
class Sim900 {
  private:
    byte powerPin;
    SoftwareSerial sim;
    bool willUseNet;
    bool debugMode;
    bool autoStart;

    void flushData() {
      while (sim.available()>0) sim.read();
    }

    void readLine(int numLines = 1, char *response = nullptr, int length = 0) {
      if (numLines < 1) return;
      char c;

      for (int i = 0; i < numLines - 1; i++) {
        c = sim.read();
        while (c < 65 || c > 90) c = sim.read();
        while (sim.read() != '\n') continue;
      }

      delay(100);
      if (response != nullptr) {

        c = sim.read();
        while (c < 65 || c > 90) c = sim.read();
        int i = 0;
        while (c != '\n' && i < length) {
          response[i] = c;
          c = sim.read();
          i++;
        }
        response[i] = '\0';
        flushData();
        // int bytesRead = sim.readBytesUntil('\n', response, length);
        // response[bytesRead] = '\0';
      } else {
        while (sim.read() != '\n') continue;
      }
    }

    bool sendCmd(const char *cmd, int numLines, const char *expected = "", int lengthExpected = 0, int numLines2 = 0) {
      flushData();
      sim.println(cmd);
      char response[lengthExpected + 1];
      readLine(numLines, response, lengthExpected);
      
      if (lengthExpected > 0 && strcmp(response, expected) != 0) {
        return false;
      }
      readLine(numLines2);
      return true;
    }

    void setup_sms() {
      while (!sendCmd("AT+CREG?", 2, "CREG: 0,1", 9)) {
        Serial.println("sms setup...");
        delay(500);
      }
    }

    void setup_net() {
      while (!sendCmd("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", 2, "OK", 2)) {
        Serial.println("gps err: AT+SAPBR=Contype");
        delay(500);
      }
      while (!sendCmd("AT+SAPBR=3,1,\"APN\",\"internet\"", 2, "OK", 2)) {
        Serial.println("gps err: AT+SAPBR=APN");
        delay(500);
      }
      while (!sendCmd("AT+SAPBR=1,1", 2, "OK", 2)) {
        Serial.println("gps err: AT+SAPBR=1,1");
        delay(500);
      }
    }

  public:

    Sim900(byte rx = 7, byte tx = 8, byte powerPin = 9, bool willUseNet = true, bool debugMode = false, bool autoStart = true): 
      sim(rx, tx), powerPin(powerPin), willUseNet(willUseNet), debugMode(debugMode), autoStart(autoStart) {}

    
    // SETUP
    void setup() {
      sim.listen(); //
      // power up device
      Serial.println("sim900 setup 1...");
      if (autoStart) {
        digitalWrite(powerPin, LOW);
        delay(2000);
        digitalWrite(powerPin, HIGH);
        delay(7000);
      }

      sim.begin(9600);
      Serial.println("sim900 setup 2...");

      if (debugMode) {
        Serial.println("debugMode sim900");
      }

      while (debugMode) {
        if(Serial.available()>0) sim.write(Serial.read());
        if(sim.available()>0) Serial.write(sim.read());
      }

      Serial.println("sim900 setup 3...");
      setup_sms();
      Serial.println("sim900 setup 4...");
      if (willUseNet) {
        setup_net();
      }
      Serial.println("sim900 setup 5...");

      Serial.println("Sim 900 initialized!");
    }


    // SMS
    void send_sms_start() {
      sim.listen();
      if (!sendCmd("AT+CMGF=1", 2, "OK", 2)) {
        Serial.println("Error on AT+CMGF=1");
        return false;
      }
      Serial.println("Will send sms");
      sim.print("AT+CMGS=\"");
      sim.print(phone);
      sim.println("\"");
      sim.readStringUntil('\n');
      sim.readStringUntil('\n');
    }

    void send_sms_message(const char *message) {
      sim.print(message);
    }

    void send_sms_end() {
      sim.write(26);
      delay(200);
      sim.println();
      delay(3000);
    }

    // HTTP
    void http_get_start() {
      sim.listen();
      while (!sendCmd("AT+HTTPINIT", 2, "OK", 2)) {
        Serial.println("http err: HTTPINIT");
        delay(250);
        sendCmd("AT+HTTPTERM", 2, "OK", 2);
        delay(250);
      }
      flushData();
      http_get_print("AT+HTTPPARA=\"URL\",\"");
    }

    void http_get_print(const char *str) {
      sim.print(str);
    }

    void http_get_end(char *response = nullptr, int length = 0) {
      http_get_print("\"");
      if (!sendCmd("", 2, "OK", 2)) {
        Serial.println("http err: HTTPPARA");
        return;
      }
      if (!sendCmd("AT+HTTPACTION=0", 2, "OK", 2)) {
        Serial.println("http err: HTTPACTION");
        return;
      }
      while (sim.read() != ':') continue;
      while (sim.read() != '\n') continue;

      if (response != nullptr) {
        sendCmd("AT+HTTPREAD", 3);
        sim.readBytesUntil('\n', response, length);
      }

      sendCmd("AT+HTTPTERM", 2);
    }
    
};



// --------------------------------------- SerialArduino
#include <SoftwareSerial.h>
typedef void (*StringArrayFunction)(String[], int);
class SerialArduino {
  private:
    SoftwareSerial ss;

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

    SerialArduino(byte rx, byte tx) : ss(rx, tx) {}

    void setup() {
      ss.listen();
      ss.begin(9600);
    }

    void print(const char *text) {
      ss.listen();
      ss.print(text);
    }

    void println(const char *text) {
      ss.listen();
      ss.println(text);
    }

    void printFloat(float data) {
      ss.listen();
      ss.print(data);
    }

    void printFloatln(float data) {
      ss.listen();
      ss.println(data);
    }

    void printInt(int data) {
      ss.listen();
      ss.print(data);
    }

    void printIntln(int data) {
      ss.listen();
      ss.println(data);
    }

    void on_receive(int receive_size, StringArrayFunction callback) {
      ss.listen();
      if (ss.available() > 0) {
        String receivedData = ss.readStringUntil('\n');
        // Serial.println("Received Data: " + receivedData);

        String values[receive_size];
        int numValues = splitString(receivedData, ',', values, receive_size);

        if (numValues == receive_size) {
          // Assuming the format is "float,float,String,float"
          callback(values, receive_size);
        }
      }

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


// --------------------------------------------------------------- CLASSES
MyServo servo(12);
Sim900 sim900(7, 8, 6, false, false, true); // rx, tx, powerPin, useNet, debugMode, autoStart
SerialArduino nodemcu(4, 5); // rx, tx
SetTimeout servoRotateSetTimeout;
LED led(3);

// --------------------------------------------------------------- VARIABLES
bool isDropping = false;
int numKills = 1;
int battery = 20;
// --------------------------------------------------------------- FUNCTIONS


void send_sms_exterminated() {
  char kills[3];
  itoa(numKills, kills, 10);
  sim900.send_sms_start();
  sim900.send_sms_message("Rat exterminated. ");
  sim900.send_sms_message("Total rats in the bin: ");
  sim900.send_sms_message(kills);
  if (numKills == 15) {
    sim900.send_sms_message(". System Disabled.");
  }
  sim900.send_sms_end();
}

void send_sms_low() {
  char batter_str[4];
  itoa(battery, batter_str, 10);
  sim900.send_sms_start();
  sim900.send_sms_message("Low Battery: ");
  sim900.send_sms_message(batter_str);
  sim900.send_sms_message("%");
  sim900.send_sms_end();
}


void isDroppingFalse() {
  switch (numKills) {
    case 1:
    case 5:
    case 10:
    case 15:
      send_sms_exterminated();
      break;
    
    default:
      break;
  }

  isDropping = false;
}

void handle_receive(String data[], int size) {
  String cmd = String(data[0]);
  
  Serial.print("Received: ");
  Serial.println(cmd);
  if (cmd.equals(String("exterminated")) && !isDropping) {
    numKills = data[1].toInt();
    Serial.print("Exterminated (");
    Serial.print(numKills);
    Serial.println(")");
    servo.rotate(servo_rotate_angle, 90, dropTimeoutMs, true); // from, to, duration, withReverse
    isDropping = true;
    servoRotateSetTimeout.start(2 * dropTimeoutMs, isDroppingFalse);
  } else if (cmd.equals(String("low"))) {
    battery = data[1].toInt();
    send_sms_low();

  }
}



// --------------------------------------------------------------- SETUP
void setup() {
  Serial.begin(9600);
  led.setup();
  led.on();
  Serial.println("Setup 1");
  sim900.setup();
  led.off();
  Serial.println("Setup 2");
  servo.setup();
  Serial.println("Setup 3");
  nodemcu.setup();
  Serial.println("SETUP FINISHED!");
}

// --------------------------------------------------------------- LOOP
void loop() {
  nodemcu.on_receive(2, handle_receive);
  servoRotateSetTimeout.loop();
  servo.loop();
}


// OTHER USE CASES
void other_use_cases() {
  
}
