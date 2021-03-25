#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "Adafruit_seesaw.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ezTime.h>

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "garden/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "garden/sub"

// Data wire is connected to GPIO14
#define ONE_WIRE_BUS 14
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1 = { 0x28, 0xFE, 0xB, 0x79, 0x97, 0x4, 0x3, 0xBF };

// Variables
unsigned long previousMillis = 0;
Timezone Eastern;

// Constants
const long interval = 300000; //milliseconds (5 minutes)
//const long interval = 20000; //milliseconds (20 seconds)

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);
Adafruit_seesaw ss;

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  int watchdog = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (watchdog < 20) {
      delay(500);
      Serial.print(".");
      watchdog++;
    }
    else {
      Serial.println("WIFI: still not connected, triggering reboot ...");
      ESP.restart();
    }
  }

  Serial.println("WIFI Connected!");
}

void connectAWS()
{
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}


void getBatteryVoltage()
{
  //Battery Voltage
  float a13_reading = analogRead(A13);
  float batt_volt = (a13_reading/4095)*2*3.3*1.1;
  //String batt_volt_data = "{ \"Voltage\":\"" + String(batt_volt, 3) + "\" }";
  
  Serial.print("BatterySensor1-Voltage: "); Serial.println(batt_volt);

  //StaticJsonDocument<200> doc;
  DynamicJsonDocument doc(1024);
  doc["sensor"] = "garden-esp32-1-BatterySensor1-Voltage";
  doc["time"] = Eastern.dateTime(ISO8601);
  doc["data"] = batt_volt;
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); 
  
  // Publish Battery Voltage
  publishMessage(jsonBuffer);
}

void getSoilMeasurements()
{
  float soil_temperature_c = ss.getTemp();
  float soil_temperature_f = (soil_temperature_c*1.8)+32;
  uint16_t soil_capacitence = ss.touchRead(0);
  //String soil_temp_capacitance = "{ \"Temperature\":\"" + String(tempC, 3) + "\", \"Capacitance\":\"" + String(capread, 3) + "\" }";

  Serial.print("SoilSensor1-Temperature: "); Serial.print(soil_temperature_f); Serial.println("*F");
  Serial.print("SoilSensor1-Capacitive: "); Serial.println(soil_capacitence);

  //Soil Capacitence
  DynamicJsonDocument doc(1024);
  doc["sensor"] = "garden-esp32-1-SoilSensor1-Capacitence";
  doc["time"] = Eastern.dateTime(ISO8601);
  doc["data"] = soil_capacitence;
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); 

  //Soil Temperature
  DynamicJsonDocument doc2(1024);
  doc2["sensor"] = "garden-esp32-1-SoilSensor1-Temperature";
  doc2["time"] = Eastern.dateTime(ISO8601);
  doc2["data"] = soil_temperature_f;
  
  char jsonBuffer2[512];
  serializeJson(doc2, jsonBuffer2);
  
  // Publish Soil Measurements
  publishMessage(jsonBuffer);   //Soil Capacitence
  publishMessage(jsonBuffer2);  //Soil Temperature
}

void getTemperatureReadings()
{
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  
  float temp_reading = sensors.getTempF(sensor1);

  Serial.print("TempSensor1-Temperature: "); Serial.print(temp_reading); Serial.println("*F");

  DynamicJsonDocument doc(1024);
  doc["sensor"] = "garden-esp32-1-TempSensor1-Temperature";
  doc["time"] = Eastern.dateTime(ISO8601);
  doc["data"] = temp_reading;
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
    
  // Publish Temperature Reading
  publishMessage(jsonBuffer);
}

void publishMessage(char* jsonBuffer)
{ 

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}


void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

//  StaticJsonDocument<200> doc;
//  deserializeJson(doc, payload);
//  const char* message = doc["message"];
}

void setup() {
  Serial.begin(9600);
  connectWiFi();
  connectAWS();

  // Sync time with NTP server
  waitForSync();
  
//  Serial.println("seesaw Soil Sensor example!");
//  
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }

  sensors.begin();
  
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save currentMillis as previousMillis
    previousMillis = currentMillis;

    //Get time
    Eastern.setLocation("America/New_York");
    Serial.println("Data/Time:" + Eastern.dateTime(ISO8601));

    //Check Sensors
    getBatteryVoltage();
    getSoilMeasurements();
    getTemperatureReadings();
    
  }

  client.loop();

}
