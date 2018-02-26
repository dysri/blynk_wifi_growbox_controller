/*
  Blynk Wifi Grow Box Controller
  
  Created: 20 February 2017
  Last edit: 25 February 2017
  Author: Dylan Sri-Jayantha

  This code is designed to be embedded in an Arduino Uno board that is controlling intake/outflow fans for a hydroponic grow chamber.
  The board has been wired to supply power to an ESP8266 module
  (see: http://www.forward.com.au/pfod/CheapWifiShield/ESP2866_01_WiFi_Shield/ESP8266_WiFi_Shield_R2.pdf)
  and connected to a DHT22 sensor and 2-module power relay for toggling power to the fans
  (see wiring at: https://github.com/dysri)

*/

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <DHT.h>
#include <DHT_U.h>

// Relay pins
int INFLOW_FAN_RELAY_PIN = 2;
int OUTFLOW_FAN_RELAY_PIN = 3;

// Fan settings
int outflowTempThreshold = 75;
unsigned long fanOnPeriod = 10000; // (10 seconds)
unsigned long fanOffPeriod = 1800000; // (30 mins)
unsigned long prevFanMillis = 0;

// Temp+Hum sensor
int DHT_DATA_PIN = 4;
DHT dht22(DHT_DATA_PIN, DHT22);

// Input Blynk Auth token here
char auth[] = "";

// Input WiFi credentials here
char ssid[] = "";
char pass[] = "";

BlynkTimer timer;

// ESP8266 Hardware Serial
#define EspSerial Serial
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);

void timerEvent(){
  
  // Take reading on DHT22
  float hum = dht22.readHumidity();
  float tempf_22 = dht22.readTemperature(true);

  // Control fan relays
  // If temp too high, outflow fan on
  if (tempf_22 > outflowTempThreshold) digitalWrite(OUTFLOW_FAN_RELAY_PIN, HIGH);
  // If temp too low, stop outflow fan
  else if (tempf_22 <= outflowTempThreshold) digitalWrite(OUTFLOW_FAN_RELAY_PIN, LOW);

  unsigned long currFanMillis = millis();
  // If box needs fresh intake, intake fan on
  if (((unsigned long)(currFanMillis - prevFanMillis) >= fanOffPeriod) && (digitalRead(INFLOW_FAN_RELAY_PIN) == LOW)) {
    digitalWrite(INFLOW_FAN_RELAY_PIN, HIGH);
    prevFanMillis = currFanMillis;
  }
  // If intake is complete, turn off intake
  else if (((unsigned long)(currFanMillis - prevFanMillis) >= fanOnPeriod) && (digitalRead(INFLOW_FAN_RELAY_PIN) == HIGH)) {
    digitalWrite(INFLOW_FAN_RELAY_PIN, LOW);
    prevFanMillis = currFanMillis;
  }
    
  // Transmit to Blynk server  
  Blynk.virtualWrite(V0, tempf_22);
  Blynk.virtualWrite(V1, hum);
}

void setup() {
  delay(1000);
  
  pinMode(INFLOW_FAN_RELAY_PIN, OUTPUT);
  pinMode(OUTFLOW_FAN_RELAY_PIN, OUTPUT);
  digitalWrite(OUTFLOW_FAN_RELAY_PIN, LOW);
  digitalWrite(INFLOW_FAN_RELAY_PIN, LOW);
  
  EspSerial.begin(ESP8266_BAUD);
  dht22.begin();

  Blynk.begin(auth, wifi, ssid, pass);
  
  timer.setInterval(10000L, timerEvent);
}

void loop() {
  Blynk.run();
  timer.run();
}
