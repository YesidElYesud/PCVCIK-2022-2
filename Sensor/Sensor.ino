#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <WiFi.h>
#include "ArduinoJson.h"
#include "AsyncUDP.h"

#define BNO055_DELAY_MS (100)

// WiFi Credentials
#define WIFI_NETWORK "XXXX"
#define WIFI_PASSWORD "XXXX"
#define WIFI_TIMEOUT_MS 20000

// UDP Server Details
#define SERVER "192.168.1.121"
#define PORT 3002


Adafruit_BNO055 sensor = Adafruit_BNO055();

float filterPOld, filterPNew = 0; // Variables de precisión, derivadas en la función de configuración

float thetaM; // Inclinación medida a partir del eje X del acelerómetro.
float thetaFOld=0; // Valor antiguo filtrado - El valor inicial es 0
float thetaFNew; // Nuevo valor filtrado, se derivará y sustituirá el valor antiguo para la siguiente iteración

float phiM;
float phiFOld=0; // Valor antiguo filtrado - El valor inicial es 0
float phiFNew; // Nuevo valor filtrado, se derivará y sustituirá el valor antiguo para la siguiente iteración

unsigned long timePrevious; // Variable para almacenar la hora antigua en milisegundos.
float timeDiff; // variable para capturar la diferencia de tiempo
float thetaG = 0; // Distancia angular en términos de ángulo en el eje x
float phiG = 0; // Distancia angular en términos de ángulo en el eje y

float roll = 0; // Filtro complementario variable
float pitch = 0; // Filtro complementario variable
float trust = 0.95; // Porcentaje de confianza para el filtro complementario del giroscopio

float yaw = 0; // Ángulo de medición de la guiñada basado en el mangetómetro
float pitchRad;
float rollRad;
float Xm;
float Ym;

String json; // Mantener la cadena json
AsyncUDP udpClient; // Async UDPClient object

void setup() {
  Serial.begin(115200);
  // Conéctate primero a WiFI
  connectToWiFi ();

  sensor.begin();
  delay(1000);
  int8_t temp=sensor.getTemp();
  sensor.setExtCrystalUse(true);
  // Derivar la precisión del filtro para el valor antiguo y el nuevo
  filterPOld = 0.95;// FILTER_PRECISION en porcentaje / 100;
  filterPNew = 1 - filterPOld;
  timePrevious = millis(); // Inicialización del tiempo en milisegundos
}

void loop() {
  // Lectura de los datos del acelerómetro 
  imu::Vector<3> acc = sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Lectura de los datos del giroscopio
  imu::Vector<3> gyr = sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Lectura de los datos del magnetómetro
  imu::Vector<3> mag = sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  uint8_t system, gyro, accel, mg = 0;
  sensor.getCalibration(&system, &gyro, &accel, &mg);
  // Cálculo de la inclinación a partir de los datos del acelerómetro eje x
  thetaM=atan2(acc.x()/9.8,acc.z()/9.8)*180/3.14159265359;
  // Calcular el nuevo valor theta a partir del valor theta medido y el valor antiguo
  thetaFNew=filterPOld*thetaFOld + filterPNew*thetaM;
  // Cálculo de la inclinación a partir de los datos del acelerómetro eje y
  phiM=atan2(acc.y()/9.8,acc.z()/9.8)*180/3.14159265359;
  // Calcular el nuevo valor de phi a partir del valor de phi medido y del valor antiguo
  phiFNew=filterPOld*phiFOld + filterPNew*phiM;

  // Evaluar la aceleración angular a partir de los datos del giroscopio
  // Cálculo de la diferencia de tiempo (en segundos)
  timeDiff = (millis() - timePrevious)/1000.0;
  timePrevious=millis(); // Reajuste de la hora con la hora actual

  thetaG = thetaG - gyr.y() * timeDiff;
  phiG = phiG + gyr.x() * timeDiff;

  // Implementación del filtro complementario estimación del balanceo
  roll=(roll + gyr.x() * timeDiff) * trust + phiM * (1 - trust);
  // Estimación del tono de la implementación del filtro complementario
  pitch=(pitch - gyr.y() * timeDiff) * trust + thetaM * (1 - trust);
  Base de medición de la guiñada en la dirección del mangetómetro
  rollRad=roll/180*3.14159265359;
  pitchRad=pitch/180*3.14159265359;
  Xm=mag.x()*cos(pitchRad)-mag.y()*sin(rollRad)*sin(pitchRad)+mag.z()*cos(rollRad)*sin(pitchRad);
  Ym=mag.y()*cos(rollRad)+mag.z()*sin(rollRad);
  yaw = atan2(Ym,Xm)*180/3.14159265359;

  Serial.print(system);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
  // Prepara el JSON
  json = GetJSONString ("hand", pitch, roll, yaw);
  Serial.println(json);
  // Envía los datos al server
  UDPSendData(json);

  thetaFOld=thetaFNew;
  phiFOld=phiFNew;
  
  delay(BNO055_DELAY_MS);

}

void connectToWiFi (){
  Serial.print ("Conectando al WiFI");
  WiFi.mode (WIFI_STA);
  WiFi.begin (WIFI_NETWORK, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis ();
  while (WiFi.status () != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
    Serial.print (".");
    delay(100);
  }
  // TODO: Error Handling
  if (WiFi.status () != WL_CONNECTED){
    Serial.print ("Fallido!");
  } else {
    Serial.print ("Conectado!");
    Serial.print (WiFi.localIP());
  }
}

String GetJSONString(String sensorName, float pitch, float roll, float yaw) 
{
    StaticJsonDocument<1024> staticJsonDocument;
    staticJsonDocument["Nombre"] = sensorName;
    staticJsonDocument["pitch"] = pitch;
    staticJsonDocument["roll"] = roll;
    staticJsonDocument["yaw"] = yaw;
    
    char docBuf[1024];
    serializeJson(staticJsonDocument, docBuf);

    return String(docBuf);
}

void UDPConnect()
{
    IPAddress ipAddress = IPAddress();
    ipAddress.fromString(SERVER);
    udpClient.connect(ipAddress, PORT);   
    Serial.println ("Server Conectado");
}

void UDPSendData(String message)
{
    char charBuffer[1024];
    message.toCharArray(charBuffer, 1024);

    if (udpClient.connected()){
      udpClient.broadcastTo(charBuffer, PORT);
    } else {
      Serial.println ("Server no conectado");
      UDPConnect();
    }
}
