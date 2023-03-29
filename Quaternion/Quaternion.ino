#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <WiFi.h>
#include "ArduinoJson.h"
#include "AsyncUDP.h"
#include "Streaming.h" 

#define BNO055_DELAY_MS (100)
#define WIFI_NETWORK "WiFi-B8E1"
#define WIFI_PASSWORD "25571806"
#define WIFI_TIMEOUT_MS 20000
#define SERVER "192.168.1.121"
#define PORT 3002

const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

Adafruit_BNO055 sensor = Adafruit_BNO055();
float roll = 0; // Complementary filter variable
float pitch = 0; // Complementary filter variable
float yaw = 0; // New Yaw measurement angle based on mangetometer

String json; // Hold json string
AsyncUDP udpClient; // Async UDPClient object

bool isCal = false;

void connectToWiFi (){
  Serial.print ("Connecting to WiFI");
  WiFi.mode (WIFI_STA);
  WiFi.begin (WIFI_NETWORK, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis ();
  while (WiFi.status () != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print (".");
    delay(100);
  }

  // TODO: Error Handling
  if (WiFi.status () != WL_CONNECTED){
    Serial.print ("Failed!");
  } else {
    Serial.print ("Connected!");
    Serial.print (WiFi.localIP());
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Connect to WiFI first
  connectToWiFi ();
  Serial.println ("I2C scanner. Scanning ...");
  Wire.begin();
  byte i = 40;
  Wire.beginTransmission (i);
  if (Wire.endTransmission () == 0){
    Serial.print ("Found address: ");
    Serial.print (i, DEC);
    Serial.print (" (0x");
    Serial.print (i, HEX);
    Serial.println (")");
  } // end of good response
  Serial.println ("Done.");
  Serial.println (" device(s).");
  if(!sensor.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  int8_t temp=sensor.getTemp();
  sensor.setExtCrystalUse(true);
}

void loop() {
  uint8_t system, gyro, accel, mg = 0;
  sensor.getCalibration(&system, &gyro, &accel, &mg);
  
  imu::Quaternion quat = sensor.getQuat();
  quat.normalize ();
  imu::Vector<3> euler = quat.toEuler();
  yaw = euler.x() * 180/M_PI;
  pitch = euler.y() * -180/M_PI;
  roll = euler.z() * -180/M_PI;

  imu::Vector<3> laccel = sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  Serial  << system << "," << accel << "," << gyro << "," << mg << "," << laccel.x() << "," << laccel.y() << "," << laccel.z() << endl;
  
  // Check Battery  
  int rawValue = analogRead(A13);
  // Reference voltage on ESP32 is 1.1V
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  Serial.println((String)"Raw:" + rawValue + " Voltage:" + voltageLevel + "V Percent: " + (batteryFraction * 100) + "%");

  json = GetJSONString ("upperarm_l", pitch, roll, yaw);
  Serial.println(json);
  UDPSendData(json);
  delay(BNO055_DELAY_MS);
}

String GetJSONString(String sensorName, float pitch, float roll, float yaw) 
{
    DynamicJsonDocument doc(1024);
    doc["name"] = sensorName;
    if (sensorName == "hand_l") roll = roll - 90;
//    if (sensorName == "foot_l") pitch = pitch + 90;
    
    DynamicJsonDocument orientation(768);
    orientation["pitch"] = pitch;
    orientation["roll"] = roll;
    orientation["yaw"] = yaw;
    doc["orientation"] = orientation;
    char docBuf[1024];
    serializeJson(doc, docBuf);
    return String(docBuf);
}

void UDPConnect()
{
    IPAddress ipAddress = IPAddress();
    ipAddress.fromString(SERVER);
    udpClient.connect(ipAddress, PORT);   
    Serial.println ("Server Connected");
}

void UDPSendData(String message)
{
    char charBuffer[1024];
    message.toCharArray(charBuffer, 1024);
    if (udpClient.connected()){
      udpClient.broadcastTo(charBuffer, PORT);
    } else {
      Serial.println ("Server Not Connected");
      UDPConnect();
    }
}
