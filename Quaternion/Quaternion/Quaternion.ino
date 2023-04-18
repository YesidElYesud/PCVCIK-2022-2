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
#define WIFI_NETWORK "SANABRIA"
#define WIFI_PASSWORD "sanabria1811"
#define WIFI_TIMEOUT_MS 20000
#define SERVER "192.168.1.121"
#define PORT 3002

const int botonA = 12;
const int botonB = 14;
const int botonC = 27;
int A = 0;
int B = 0;
int C = 0;


const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // El voltaje LiPoly máximo de una batería 3.7 es 4.2

Adafruit_BNO055 sensor = Adafruit_BNO055();
float roll = 0; // Ángulo de medición de balanceo basado en mangetómetro
float pitch = 0; // Ángulo de medición de cabeceo basado en mangetómetro
float yaw = 0; // Ángulo de medición de guiñada basado en mangetómetro



String json; // Mantener json string
AsyncUDP udpClient; // Async UDPClient objeto

bool isCal = false;

void connectToWiFi (){
  Serial.print ("Conexión a WiFI");
  WiFi.mode (WIFI_STA);
  WiFi.begin (WIFI_NETWORK, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis ();
  while (WiFi.status () != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print (".");
    delay(100);
  }

  // TODO: Tratamiento de errores
  if (WiFi.status () != WL_CONNECTED){
    Serial.print ("Fallido!");
  } else {
    Serial.print ("Conectado!");
    Serial.print (WiFi.localIP());
  }
}

void setup() {
  Serial.begin(115200);

  // Definir Botones, pines de entrada
  pinMode(botonA,INPUT);
  pinMode(botonB,INPUT);
  pinMode(botonC,INPUT);

  // Conectarse primero a WiFI
  connectToWiFi ();

  Serial.println ("I2C scaner. Scaneando ...");
  Wire.begin();
  byte i = 40;
  Wire.beginTransmission (i);
  if (Wire.endTransmission () == 0){
    Serial.print ("Dirección Encontrada : ");
    Serial.print (i, DEC);
    Serial.print (" (0x");
    Serial.print (i, HEX);
    Serial.println (")");
  } // Posibles Repuestas
  Serial.println ("Listo.");
  Serial.println (" dispositivo(s).");
  if(!sensor.begin()){
    Serial.print("Ooops, no se ha detectado BNO055 ... ¡Compruebe su cableado o I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  int8_t temp=sensor.getTemp();
  sensor.setExtCrystalUse(true);
}

void loop() {
  A = digitalRead(botonA);
  B = digitalRead(botonB);
  C = digitalRead(botonC);
/*
  if (A == HIGH){
    Ab = true;
  }
    else
    {
    Ab = false;
  }
 */
  uint8_t system, gyro, accel, mg = 0;
  sensor.getCalibration(&system, &gyro, &accel, &mg);
  
  imu::Quaternion quat = sensor.getQuat();
  quat.normalize ();
  imu::Vector<3> euler = quat.toEuler();
  yaw = euler.x() * 180/M_PI;
  pitch = euler.y() * -180/M_PI;
  roll = euler.z() * -180/M_PI;

  imu::Vector<3> laccel = sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  Serial  << system << "," << accel << "," << gyro << "," << mg << "," << laccel.x() << "," << laccel.y() << "," << laccel.z() << ", Botones : A=" << A << ", B=" << B << ", C=" << C << endl;
  // Serial.println((String)"Botones : A="+A+", B="+B+", C="+C);

  // Revisar la Bateria  
  int rawValue = analogRead(13);
  // El Voltaje de referencia en ESP32 es 1.1V
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calcular el nivel de tensión
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  Serial.println((String)"Sin procesar:" + rawValue + " Voltaje:" + voltageLevel + "V Porcentaje: " + (batteryFraction * 100) + "%");

  json = GetJSONString ("hand", pitch, roll, yaw, A, B, C);
  Serial.println(json);
  UDPSendData(json);
  delay(BNO055_DELAY_MS);
}

String GetJSONString(String sensorName, float pitch, float roll, float yaw, int A, int B, int C) 
{
    DynamicJsonDocument doc(1024);
    doc["name"] = sensorName;
    if (sensorName == "hand_l") roll = roll - 90;
//    if (sensorName == "foot_l") pitch = pitch + 90;
    
    DynamicJsonDocument orientation(768);
    orientation["pitch"] = pitch;
    orientation["roll"] = roll;
    orientation["yaw"] = yaw;
    orientation["A"] = A;
    orientation["B"] = B;
    orientation["C"] = C;
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
    Serial.println ("Servidor conectado");
}

void UDPSendData(String message)
{
    char charBuffer[1024];
    message.toCharArray(charBuffer, 1024);
    if (udpClient.connected()){
      udpClient.broadcastTo(charBuffer, PORT);
    } else {
      Serial.println ("Servidor NO conectado");
      UDPConnect();
    }
}
