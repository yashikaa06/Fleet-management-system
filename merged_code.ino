#include <DHT.h>
#include <TinyGPS++.h>            // Include TinyGPS++ library
#include <WiFiNINA.h>              // Include WiFi library for Nano 33 IoT
#include <PubSubClient.h>          // Include MQTT library for Nano 33 IoT

// DHT22 Configuration
#define DHTPIN 4                   // Pin connected to Data pin of the DHT22 sensor
#define DHTTYPE DHT22              // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);          // Initialize DHT sensor

// GPS Configuration
TinyGPSPlus gps;                   // Create GPS object

// WiFi and MQTT Configuration
const char* ssid = "yashika";
const char* password = "9416306106";
const char* mqtt_server = "broker.hivemq.com";  
const int mqtt_port = 1883;  // Default MQTT port for HiveMQ

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  // Serial Setup
  Serial.begin(9600);
  
  // Initialize DHT sensor
  dht.begin();
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker (HiveMQ)
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    if (client.connect("ArduinoNano33IoT")) {
      Serial.println("Connected to MQTT broker (HiveMQ)");
    } else {
      Serial.print("Failed MQTT connection, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
  
  Serial.println("Starting...");

  // Begin hardware serial for GPS (using UART2 pins: 9 - TX, 10 - RX)
  Serial1.begin(9600);  // Use Serial1 (pins 9 and 10) for GPS communication
}

void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    while (!client.connected()) {
      if (client.connect("ArduinoNano33IoT")) {
        Serial.println("Reconnected to MQTT broker (HiveMQ)");
      } else {
        delay(2000);
      }
    }
  }
  client.loop();

  // Read temperature and humidity from the DHT22 sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if the readings are valid, else display an error
  if (isnan(humidity) || isnan(temperature)) {
    client.publish("sensor/error", "Failed to read from DHT sensor!");
  } else {
    String data = "Temperature: " + String(temperature) + " °C, Humidity: " + String(humidity) + " %";
    client.publish("sensor/data", data.c_str());  // Send data via MQTT
  }

  // Read GPS data and parse
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Send GPS location if available
  if (gps.location.isValid()) {
    String gpsData = "Latitude: " + String(gps.location.lat(), 6) + ", Longitude: " + String(gps.location.lng(), 6);
    client.publish("gps/data", gpsData.c_str());  // Send GPS data via MQTT
  } else {
    client.publish("gps/error", "Waiting for GPS signal...");
  }

  // Print the number of satellites
  if (gps.satellites.isValid()) {
    String satelliteData = "Satellites: " + String(gps.satellites.value());
    client.publish("gps/satellites", satelliteData.c_str());  // Send satellite data via MQTT
  } else {
    client.publish("gps/error", "No satellites data available.");
  }

  // Wait before the next reading
  delay(2000);
}