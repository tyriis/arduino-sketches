/*
  This a simple example of the aREST Library for Arduino (Uno/Mega/Due/Teensy)
  using the Serial port. See the README file for more details.

  Written in 2014 by Marco Schwartz under a GPL license.
*/

// Libraries
#include <SPI.h>
#include <aREST.h>
#include <avr/wdt.h>
#include <Wire.h>
#include "SparkFunHTU21D.h"
#include <ArduinoJson.h>

class PIR {

  public:
    int state = LOW;

    int led;
    int pin;
    int val = 0;

  public:
  PIR(int ledPin, int inputPin) {

    led = ledPin;
    pin = inputPin;

  }

  void Init() {
    pinMode(led, OUTPUT);      // declare LED as output
    pinMode(pin, INPUT);
  }

  void Update() {
    val = digitalRead(pin);  // read input value
    if (val == HIGH) {            // check if the input is HIGH
      digitalWrite(led, HIGH);  // turn LED ON
      if (state == LOW) {
        // we have just turned on
        publish(1);
        // We only want to print on the output change, not state
        state = HIGH;
      }
    } else {
      digitalWrite(led, LOW); // turn LED OFF
      if (state == HIGH){
        // we have just turned of
        publish(0);
        // We only want to print on the output change, not state
        state = LOW;
      }
    }
  }

  void publish(int value) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["sensor"] = "PIR";
    root["time"] = millis();
    root["value"] = value;
    char buffer[256];
    root.printTo(buffer, sizeof(buffer));
    Serial.println(buffer);
  }

};

class HTU21Reader {

  public:
    float temperature;
  public:
    float humidity;
    
  HTU21D htu;
  long intervall;
  unsigned long previousMillis;

  public:
  HTU21Reader(long readIntervall) {

    htu.begin();

    intervall = readIntervall;
    
    previousMillis = readIntervall;

  }

  void Update() {

    unsigned long currentMillis = millis();
     
    if (currentMillis - previousMillis >= intervall) {
      
      previousMillis = currentMillis;
      
      float _temperature = floor((htu.readTemperature()*2)+0.5)/2;
      if (_temperature != temperature) {
        temperature = _temperature;
        publish("temperature", temperature);
      }
      float _humidity = floor((htu.readHumidity()*2)+0.5)/2;
      if (_humidity != humidity) {
        humidity = _humidity;
        publish("humidity", humidity);
      }
    }
  }

  void publish(String key, float value) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["sensor"] = key;
    root["time"] = millis();
    root["value"] = value;
    char buffer[256];
    root.printTo(buffer, sizeof(buffer));
    Serial.println(buffer);
  }

};

// TODO write own serial handler that accept JSON input
aREST rest = aREST();
HTU21Reader htu21(10000);
PIR pir(13, 2);

void setup(void) {
  // Start Serial
  Serial.begin(115200);
  pir.Init();
  
  // Init variables and expose them to REST API
  rest.variable("temperature",&htu21.temperature);
  rest.variable("humidity",&htu21.humidity);

  // Function to be exposed
  // rest.function("led",ledControl);

  // Give name and ID to device (ID should be 6 characters long)
  // restHandler.rest.set_id("2");
  // restHandler.rest.set_name("serial");

  // Start watchdog
  wdt_enable(WDTO_4S);
  // pinMode(LED_BUILTIN, OUTPUT);
  publishStart();
  
}

void loop() {
  // Handle REST calls
  rest.handle(Serial);
  wdt_reset();
  htu21.Update();
  pir.Update();
}

void publishStart() {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["event"] = "listen";
    root["time"] = millis();
    root["value"] = "serial";
    char buffer[256];
    root.printTo(buffer, sizeof(buffer));
    Serial.println(buffer);
  }

