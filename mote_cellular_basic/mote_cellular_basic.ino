#include <Adafruit_SleepyDog.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoLowPower.h>
#include <ArduinoMqttClient_JH.h>
#include <MKRNB_JH.h>

#include "certificate.h"

#define HOSTNAME "device" // must be unique for all motes on network
#define TOPIC_OUTGOING "device/outgoing" // measurements published on this topic
#define TOPIC_INCOMING "device/incoming" // measurements published on this topic
#define CERT DEVICE_CERT

// NB and MQTT settings:
#define PINNUMBER ""
#define BROKER "**broker**.iot.us-east-2.amazonaws.com"
#define NB_TIMEOUT 60 * 1000UL
#define MQTT_TIMEOUT 30 * 1000UL

// Other settings:
#define SLEEP_PERIOD 1 * 60000 // 1 minute
#define WATCHDOG_MS 16000
#define SERIAL_BAUD 112500


// Create objects:
GPRS            gprs;
NB              nbAccess;
NBClient        client;
NBScanner       scannerNetworks;
BearSSLClient   sslClient(client);   // Used for SSL/TLS connection, integrates with ECC508
MqttClient      mqttClient(sslClient);

//=============================================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(5000);
  Serial.println(F("------------------------------------------------------------------"));
  Serial.print(HOSTNAME);
  Serial.println(F(" : 2022/09/08 Build - Basic, always asleep"));
  Serial.println(F("------------------------------------------------------------------"));

  configureUnusedPins(); // save power
  
  // Reset modem:
  nbAccess.hardReset();
  delay(5000);
  softReset();

  // Enable watchdog:
  Watchdog.enable(WATCHDOG_MS);
  Serial.println("Watchdog enabled");
  Watchdog.reset();

  // Set timeouts:
  nbAccess.setTimeout(NB_TIMEOUT);
  client.setClientTimeout(NB_TIMEOUT); // this requires edits to the NBClient class
  gprs.setTimeout(NB_TIMEOUT);
  mqttClient.setConnectionTimeout(MQTT_TIMEOUT);

  // Configure SSL and MQTT:
  ArduinoBearSSL.onGetTime(getTime);
  sslClient.setEccSlot(0, CERT);
  mqttClient.onMessage(messageReceived);

  // Connect to NB and MQTT:
  verifyConnection();
}

//=============================================================================================

void loop() {  
  // Disconnect and sleep:
    Serial.println(F("Disconnecting from MQTT..."));
    mqttClient.stop();
    Serial.println(F("Disconnecting from NB..."));
    nbAccess.shutdown();
    Serial.println(F("NB disconnected"));
    delay(100);
    Serial.end();
    configurePins(true); // alter pin mode to save power
    Watchdog.disable();

    LowPower.deepSleep(SLEEP_PERIOD);
    
    Serial.begin(SERIAL_BAUD);
    delay(2000);
    Watchdog.enable(WATCHDOG_MS);
    Watchdog.reset();
    configurePins(false); // alter pin mode to allow I/O functionality of desired pins

    // Connect to NB and MQTT:
    verifyConnection();

    // Do something here, take measurement, check shadow, etc.
} 

//=============================================================================================
/*
   Set unused pins to INPUT_PULLUP to minimize power consumption. Do not change
   D6 since the orange LED is tied to this pin.
*/
void configureUnusedPins() {
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
}

//=============================================================================================
/*
   Set pins to INPUT_PULLUP to minimize power consumption when sleeping.
   Restore their functionality to OUTPUT when not sleeping.
*/
void configurePins(bool sleep) {
  if (sleep) {
    pinMode(7, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(A0, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    pinMode(SDA, INPUT_PULLUP);
    pinMode(MISO, INPUT_PULLUP);
    pinMode(SCK, INPUT_PULLUP);
    pinMode(MOSI, INPUT_PULLUP);
  } else {
    pinMode(7, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(SCL, OUTPUT);
    pinMode(SDA, OUTPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
  }
}

//=============================================================================================
/*
   Get UNIX time in seconds from cellular module. Disregards sub-second information.
   For more precise implementation, see syncClock below.
*/
unsigned long getTime() {
  unsigned long t = nbAccess.getTime();
  if (t == 0) { // something went wrong, try again
    delay(50);
    return nbAccess.getTime();
  }
  return t;
}

//=============================================================================================
/*
   If connection is not established, attempt to connect both NB and MQTT until
   successful.
*/
void verifyConnection() {
  byte mqttFailures = 0; // if NB repeatedly connects but MQTT repeatedly fails, hard reset

  while (!mqttClient.connected()) {
    
    Watchdog.reset();
    byte attemptCount = 0;
    while (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
      if (attemptCount > 4) {
        // Reset modem:
        nbAccess.hardReset();
        delay(1000);
        attemptCount = 0;
      } else {
        attemptCount++;
      }
      connectNB();
    }

    // Connect to MQTT:
    delay(5000);
    Watchdog.reset();
    if (!connectMQTT()) {
      mqttFailures++;
    }
    Watchdog.reset();

    if (mqttFailures > 3) {
      Serial.println(F("More than 4 failed attempts to connect MQTT, hard resetting"));
      Watchdog.reset();
      nbAccess.hardReset();
      delay(1000);
      connectNB(); // force NB reconnect
      mqttFailures = 0;
    }
  }
}

//=============================================================================================
/*
   Connects to narrowband (LTE) cellular network.
*/
void connectNB() {
  Serial.println(F("Attempting to connect to cellular network"));

  if ((nbAccess.begin(PINNUMBER) == NB_READY) && (gprs.attachGPRS() == GPRS_READY)) {
    Serial.println(F("NB success"));
  } else {
    Serial.println(F("NB connection failed"));
    delay(500);
  }
}

//=============================================================================================
/*
   Connects to MQTT broker (AWS in this case) to which measurements are streamed
   and shadow is held.
*/
bool connectMQTT() {
  Serial.println(F("Attempting to connect to MQTT broker"));
  if (mqttClient.connect(BROKER, 8883)) {
    Serial.println(F("MQTT connected"));
    mqttClient.subscribe(TOPIC_INCOMING, 0);
    return true;
  }
  Serial.println(F("MQTT failed to connect"));
  return false;
}

//=============================================================================================
/*
   Soft resets the SARA modem without having to create a modem class instance.
*/
void softReset() {
  SerialSARA.begin(115200);
  delay(10);
  SerialSARA.println(F("AT+CFUN=15"));
  delay(5000);
  Serial.println(F("Modem was soft reset"));
}

//=============================================================================================
/*
   This function is triggered when a message is received on the topic
   TOPIC_SHADOW_IN.
*/
void messageReceived(int messageSize) {
  // Read the message:
  char msgIn[55];
  byte i = 0;
  while (mqttClient.available()) {
    if (i < 55) {
      msgIn[i] = (char)mqttClient.read();
      i++;
    } else {
      mqttClient.read();
    }
  }

  Serial.println(F("Message received:"));
  Serial.println(msgIn);
}
