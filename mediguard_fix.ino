#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <TinyGPS++.h>

#define DHT_SENSOR_PIN  4 // ESP32 pin GPIO04 connected to DHT11 sensor
#define DHT_SENSOR_TYPE DHT11
#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600

// Replace the next variables with your SSID/Password combination
const char* ssid = "WIFI_USERNAME";
const char* password = "WIFI_PASSWORD";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
String clientId = "MediGuard12345678";
// const char* mqttUser = "ardhanurfan";
// const char* mqttPassword = "adafruit io key";
// const char* mqttTopic = "ardhanurfan/feeds/mediguard";


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

// LED Pin
const int ledPin = 13;
const int led = 2;

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
TinyGPSPlus gps;  // the TinyGPS++ object

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPS_BAUDRATE);
  dht_sensor.begin(); // initialize the DHT sensor

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
  pinMode(led, OUTPUT);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, messageTemp);

  String command = String(doc["lock"].as<const char*>());
  String device = String(doc["device"].as<const char*>());

  // If a message is received on the topic MediGuardDevice/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "MediGuardDevice/output" && device == clientId) {
    Serial.print("Changing output to ");
    if(command == "true"){
      Serial.println("on");
      digitalWrite(led, HIGH);
    }
    else if(command == "false"){
      Serial.println("off");
      digitalWrite(led, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        Serial.println(clientId.c_str());
        // Subscribe
        client.subscribe("MediGuardDevice/output");
    } else {
        Serial.print("failed with state  ");
        Serial.println(client.state());
        delay(2000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    
    // read temperature in Celsius
    float tempC = dht_sensor.readTemperature();
    // read humidity
    float humi  = dht_sensor.readHumidity();

    // Convert the value to a char array
    char tempString[8];
    dtostrf(tempC, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    // Convert the value to a char array
    char humiString[8];
    dtostrf(humi, 1, 2, humiString);
    Serial.print("humidity: ");
    Serial.println(humiString);
  
    DynamicJsonDocument doc(1024);
    doc["device"] = clientId;
    doc["temperature"] = tempC;
    doc["humidity"] = humi;

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish("MediGuardDevice/data", buffer, n);

    // Menghidupkan LED jika suhu di atas ambang tertentu (misalnya, 30°C)
    if (tempC > 28.0) {
      digitalWrite(ledPin, HIGH); // Menghidupkan LED
    } else if (tempC <= 28.0) {
      digitalWrite(ledPin, LOW); // Mematikan LED
    }

    // Read GPS data
    if (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (gps.location.isValid()) {
          Serial.print(F("latitude: "));
          Serial.println(gps.location.lat());

          Serial.print(F("longitude: "));
          Serial.println(gps.location.lng());

          Serial.print(F("altitude: "));
          if (gps.altitude.isValid())
            Serial.println(gps.altitude.meters());
          else
            Serial.println(F("INVALID"));

          Serial.print(F("speed: "));
          if (gps.speed.isValid()) {
            Serial.print(gps.speed.kmph());
            Serial.println(F(" km/h"));
          } else {
            Serial.println(F("INVALID"));
          }

          // Convert GPS data to a char array
          char latString[12];
          dtostrf(gps.location.lat(), 1, 6, latString);

          char lonString[12];
          dtostrf(gps.location.lng(), 1, 6, lonString);

          char altString[12];
          if (gps.altitude.isValid())
            dtostrf(gps.altitude.meters(), 1, 2, altString);
          else
            strcpy(altString, "INVALID");

          char speedString[8];
          if (gps.speed.isValid())
            dtostrf(gps.speed.kmph(), 1, 2, speedString);
          else
            strcpy(speedString, "INVALID");

          doc["latitude"] = latString;
          doc["longitude"] = lonString;
          doc["altitude"] = altString;
          doc["speed"] = speedString;

          char buffer[256];
          size_t n = serializeJson(doc, buffer);
          client.publish("MediGuardDevice/data", buffer, n);
          delay(1000);
        }
      }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));
    delay(1000);
  }
}
