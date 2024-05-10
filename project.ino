#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"

/************************* DHT Sensor Setup *********************************/

#define DHTPIN   D2     // Digital pin connected to the DHT sensor
#define DHTTYPE  DHT11  // Type of DHT Sensor, DHT11 or DHT22
DHT dht(DHTPIN, DHTTYPE);

#define pumpout   D3     // Connect LED on pin D3
#define aiout   D1     // Connect LED on pin D1

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "free"   
#define WLAN_PASS       "ktxa18free"   

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                                 
#define AIO_USERNAME    "minhpham51"                        
#define AIO_KEY         ""   

/************ Global State  ******************/
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup feeds called 'temp' & 'hum' for publishing.

Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/cambien1"); 
Adafruit_MQTT_Publish hum = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/cambien3");

// controlling pump, automode, ai
Adafruit_MQTT_Subscribe pumpin = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nutnhan1");
Adafruit_MQTT_Subscribe automode = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/nutnhan2");
Adafruit_MQTT_Subscribe aiin = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ai");

void MQTT_connect();

int counter;
float lastHumidity = -1;
float lastTemperature = -1;
void setup() {
  Serial.begin(115200);
  delay(10);
  pinMode(pumpout, OUTPUT);
  pinMode(aiout, OUTPUT);
  Serial.println(F("Adafruit MQTT"));

  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  mqtt.subscribe(&pumpin);
  mqtt.subscribe(&automode);
  mqtt.subscribe(&aiin);
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  MQTT_connect();

  Adafruit_MQTT_Subscribe *subscription;

  while ((subscription = mqtt.readSubscription(200))) {
    if (subscription == &pumpin) {
      Serial.print(F("Got Pump: "));
      char *pumpstate = (char *)pumpin.lastread;
      Serial.println(pumpstate);
      String message = String(pumpstate);
      message.trim();
      if (message == "1") {
        digitalWrite(pumpout, HIGH);
        Serial.println("PUMP ON");
      }
      if (message == "0") {
        digitalWrite(pumpout, LOW);
        Serial.println("PUMP OFF");
      }
    }
    if (subscription == &aiin) {
      Serial.print(F("Got AI: "));
      char *aistate = (char *)aiin.lastread;
      Serial.println(aistate);
      String message = String(aistate);
      message.trim();
      if (message == "Bad") {
        digitalWrite(aiout, HIGH);
        Serial.println("Bad");
      }
      else if (message == "Good") {
        digitalWrite(aiout, LOW);
        Serial.println("Good");
      }
      else{
        digitalWrite(aiout, LOW);
        Serial.println("No leaf");
      }
    }
    if (subscription == &automode) {
      Serial.print(F("Got auto mode: "));
      char *automodestate = (char *)automode.lastread;
      Serial.println(automodestate);
      String message = String(automodestate);
      message.trim();
      if (message == "1") {
        if(h > 70)
        digitalWrite(pumpout, HIGH);
        delay(5000);
        digitalWrite(pumpout, LOW);
        Serial.println("High temperature");
      }
    }
  }

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));

  // publish humi and temp
  if(abs(lastHumidity - h) >= 1){
    Serial.print(F("\nSending Humidity val "));
    if (! hum.publish(h)) {
        Serial.println(F("Failed"));
    } else {
        Serial.println(F("OK!"));
    }
    lastHumidity = h;
  }
  if(abs(lastTemperature - t) >= 1){
    Serial.print(F("\nSending Temperature val "));
    if (! temp.publish(t)) {
        Serial.println(F("Failed"));
    } else {
        Serial.println(F("OK!"));
    }
    lastTemperature = t;
  }

}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}