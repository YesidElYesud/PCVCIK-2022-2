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
#define WIFI_NETWORK "yesud"
#define WIFI_PASSWORD "yeye2000"
#define WIFI_TIMEOUT_MS 20000
#define SERVER "192.168.1.121"
#define PORT 3002

// Declaración de variables
// Periodo de muestro
double t, t0 = 0;
const float Ts = 1; //Periodo de muestreo en microsegundos

// Botones
const int botonA = 12;
const int botonB = 14;
const int botonC = 27;
const int botonD = 26;
int A = 0;
int B = 0;
int C = 0;
int D = 0;

// Nivel Bateria
const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // El voltaje LiPoly máximo de una batería 3.7 es 4.2

//Sensor IMU BNO055
Adafruit_BNO055 sensor = Adafruit_BNO055();
float roll = 0; // Ángulo de medición de balanceo basado en mangetómetro
float pitch = 0; // Ángulo de medición de cabeceo basado en mangetómetro
float yaw = 0; // Ángulo de medición de guiñada basado en mangetómetro

//Json objeto
String json;

// Async UDPClient objeto
AsyncUDP udpClient; 

// Variable boolena
bool isCal = false;
bool WifiReady = false;
bool I2CReady = false;
bool leidoBoton = false;
bool leidoIMU = false;

// Maquina de estados
enum State 
{
  Inicio,
  Lectura,
  Procesamiento,
  Transmision
};

// Cambiador de estados
int conectado = 0;
int proceso = 0;

State currentState;

//Arranca Nayro Quintana
void setup() {
  // Declaración de pines ESP32
  // Entradas
  pinMode(botonA,INPUT_PULLDOWN);
  pinMode(botonB,INPUT_PULLDOWN);
  pinMode(botonC,INPUT_PULLDOWN);
  pinMode(botonD,INPUT_PULLDOWN);

  //Reloj externo
  sensor.setExtCrystalUse(true);

  // Iniciar puertos seriales
  Serial.begin(115200);

  // Inicio sistema
  currentState = Inicio;
}

void loop() {
  //t = micros();
  updateStateMachine();

  if (t < t0)
  {
    t0 = 0;
  }
  
  if (t - t0 >= Ts)
  {   
    //updateStateMachine();  
    t0 = t;        
  } 

}

// Funciones

// Maquina de Estados

void updateStateMachine()
{
  switch (currentState)
  {
    case Inicio: stateInicio(); break;
    case Lectura: stateLectura(); break;
    case Procesamiento: stateProcesamiento(); break;
    case Transmision: stateTransmision(); break;
  }  
}

// Transiciones
void stateInicio()
{
  if (conectado == 0)
    changeState(State::Inicio);
  if (conectado == 1)
    changeState(State::Lectura);
}

void stateLectura()
{
  if (conectado == 0)
    changeState(State::Inicio);
  if (proceso == 0)
    changeState(State::Lectura);
  if (proceso == 1)
    changeState(State::Procesamiento);
}

void stateProcesamiento()
{
  if (conectado == 0)
    changeState(State::Inicio);
  if (proceso == 0)
    changeState(State::Lectura);
  if (proceso == 1)
    changeState(State::Procesamiento);
  if (proceso == 2)
    changeState(State::Transmision);
}

void stateTransmision()
{
  if (conectado == 0)
    changeState(State::Inicio);
  if (proceso == 0)
    changeState(State::Lectura);
  if (proceso == 1)
    changeState(State::Procesamiento);
  if (proceso == 2)
    changeState(State::Transmision);
}

void changeState(State newState)
{  
  currentState = newState; 
  switch (currentState)
  {
    case State::Inicio: outputInicio();   break;
    case State::Lectura: outputLectura();   break;
    case State::Procesamiento: outputProcesamiento();   break;
    case State::Transmision: outputTransmision();   break;
    default: break;
  }
}

// Funciones estados
void outputInicio()
{ 
  connectToWiFi ();
  protocoI2C ();
  delay(1000);
  if (WifiReady == true && I2CReady == true)
    conectado = 1;
  Serial.println ("Conexión");
  Serial.println (conectado);
}

void outputLectura()
{ 
  LecturaBotones();
  lecturaBno055();
 // bateriaRevision();
  if (leidoBoton == true && leidoIMU == true)
    proceso = 1;
    leidoBoton = false;
    leidoIMU = false;
}

void outputProcesamiento()
{ 
  json = GetJSONString ("Sensor", pitch, roll, yaw, A, B, C, D);
  Serial.println(json);
  proceso = 2;
}

void outputTransmision()
{ 
  UDPSendData(json);
  // delay(BNO055_DELAY_MS);
  proceso = 0;
}

// Conectarse  a WiFI
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
    WifiReady = true;
    Serial.println ("Wifi= ");
    Serial.println (WifiReady);
  }
}

//
void protocoI2C (){
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
  I2CReady = true;
  Serial.println ("I2C = ");
  Serial.println (I2CReady);
  if(!sensor.begin()){
    Serial.print("Ooops, no se ha detectado BNO055 ... ¡Compruebe su cableado o I2C ADDR!");
    while(1);
  }
}

void LecturaBotones (){
  A = digitalRead(botonA);
  B = digitalRead(botonB);
  C = digitalRead(botonC);
  D = digitalRead(botonD);
  leidoBoton = true;
}


void lecturaBno055 (){
  uint8_t system, gyro, accel, mg = 0;
  sensor.getCalibration(&system, &gyro, &accel, &mg);
  
  imu::Quaternion quat = sensor.getQuat();
  quat.normalize ();
  imu::Vector<3> euler = quat.toEuler();
  yaw = euler.x() * 180/M_PI;
  pitch = euler.y() * -180/M_PI;
  roll = euler.z() * -180/M_PI;

  imu::Vector<3> laccel = sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //Serial  << system << "," << accel << "," << gyro << "," << mg << "," << laccel.x() << "," << laccel.y() << "," << laccel.z() << ", Botones : A=" << A << ", B=" << B << ", C=" << C << endl;

  leidoIMU = true;
}

void bateriaRevision (){
  int rawValue = analogRead(32);
  // El Voltaje de referencia en ESP32 es 1.1V
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calcular el nivel de tensión
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  Serial.println((String)"Sin procesar:" + rawValue + " Voltaje:" + voltageLevel + "V Porcentaje: " + (batteryFraction * 100) + "%");
}

String GetJSONString(String sensorName, float pitch, float roll, float yaw, int A, int B, int C,int D) 
{
    DynamicJsonDocument doc(1024);
    doc["name"] = sensorName;
    
    DynamicJsonDocument orientation(768);
    orientation["pitch"] = pitch;
    orientation["roll"] = roll;
    orientation["yaw"] = yaw;
    orientation["A"] = A;
    orientation["B"] = B;
    orientation["C"] = C;
    orientation["D"] = D;
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