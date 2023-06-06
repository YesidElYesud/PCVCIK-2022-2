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

// Variables del filtro de Kalman
float dt = 0.01; // Intervalo de tiempo entre mediciones (segundos)
float A[2][2] = {{1, dt}, {0, 1}}; // Matriz de transición de estado (cambia el signo de -dt a dt)
float H[2][2] = {{1, 0}, {0, 1}}; // Matriz de observación (cambia el tamaño de 1x2 a 2x2)
float Q[2][2] = {{0.001, 0}, {0, 0.001}}; // Matriz de ruido del proceso
float R[2][2] = {{0.5, 0}, {0, 0.5}}; // Matriz de ruido de la medición (cambia el tamaño de 1x1 a 2x2 y agrega varianza para la velocidad angular)
float P[2][2] = {{1, 0}, {0, 1}}; // Matriz de covarianza inicial
float x[2] = {0, 0}; // Estado inicial (ángulo y velocidad angular)

// Periodo de muestro
double t, t0 = 0;
const float Ts = 50000; //Periodo de muestreo en microsegundos

// Botones
const int botonA = 12;
const int botonB = 14;
const int botonC = 27;
const int botonD = 26;
int Ab = 0;
int Bb = 0;
int Cb = 0;
int Db = 0;


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

  // Inicio periodo de muestreo
  t0 = micros();
}

void loop() {
  t = micros();
  //updateStateMachine();
  if (t < t0)
  {
    t0 = 0;
  }
  
  if (t - t0 >= Ts)
  {   
    updateStateMachine();
    Serial.print("Ts: "); 
    Serial.print((t - t0)/1000); 
    Serial.println("[ms]");  
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
  json = GetJSONString ("Sensor", pitch, roll, yaw, Ab, Bb, Cb, Db);
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
  Ab = digitalRead(botonA);
  Bb = digitalRead(botonB);
  Cb = digitalRead(botonC);
  Db = digitalRead(botonD);
  leidoBoton = true;
}


void lecturaBno055 (){
  /*
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
*/

imu::Quaternion quat = sensor.getQuat();
  quat.normalize ();
  imu::Vector<3> eulera = quat.toEuler();
  yaw = eulera.x() * 180/M_PI;
  pitch = eulera.y() * -180/M_PI;
  roll = eulera.z() * -180/M_PI;

  imu::Vector<3> euler = sensor.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Crear el vector de medición Z
  float Z[2] = {euler.x(), gyro.z()};

  // Actualizar el filtro de Kalman
  kalman_update(Z);

  // Imprimir el ángulo y la velocidad angular estimados
  Serial.print("Ángulo: "); Serial.print(x[0]);
  Serial.print("°, Velocidad angular: "); Serial.print(x[1]);
  Serial.println("°/s");
  


  leidoIMU = true;
}

void bateriaRevision (){
  int rawValue = analogRead(32);
  // El Voltaje de referencia en ESP32 es 1.1V
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calcular el nivel de tensión
  float batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE;
  Serial.println((String)"Sin procesar:" + rawValue + " Voltaje:" + voltageLevel + "V Porcentaje: " + (batteryFraction * 100) + "%");
}

String GetJSONString(String sensorName, float pitch, float roll, float yaw, int Ab, int Bb, int Cb,int Db) 
{
    DynamicJsonDocument doc(1024);
    doc["name"] = sensorName;
    
    DynamicJsonDocument orientation(768);
    orientation["pitch"] = pitch;
    orientation["roll"] = roll;
    orientation["yaw"] = yaw;
    orientation["A"] = Ab;
    orientation["B"] = Bb;
    orientation["C"] = Cb;
    orientation["D"] = Db;
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
// Función de multiplicación de matrices
void matmul(float A[][2], float B[][2], float C[][2], int m, int n, int p) {
  float temp[m][p];
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < p; j++) {
      temp[i][j] = 0;
      for (int k = 0; k < n; k++) {
        temp[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < p; j++) {
      C[i][j] = temp[i][j];
    }
  }
}

// Función de actualización del filtro de Kalman
void kalman_update(float Z[2]) {
  // Paso de predicción
  float x_pred[2] = {A[0][0] * x[0] + A[0][1] * x[1], A[1][0] * x[0] + A[1][1] * x[1]};
  float P_pred[2][2];
  float A_P[2][2];
  matmul(A, P, A_P, 2, 2, 2);
  
  // Calcular la traspuesta de A
  float A_T[2][2] = {{A[0][0], A[1][0]}, {A[0][1], A[1][1]}};
  
  // Calcular P_pred = A * P * A^T + Q
  matmul(A_P, A_T, P_pred, 2, 2, 2);
  P_pred[0][0] += Q[0][0]; P_pred[0][1] += Q[0][1];
  P_pred[1][0] += Q[1][0]; P_pred[1][1] += Q[1][1];

  // Paso de actualización
  float y[2] = {Z[0] - (H[0][0] * x_pred[0] + H[0][1] * x_pred[1]), Z[1] - (H[1][0] * x_pred[0] + H[1][1] * x_pred[1])};
  float S[2][2];
  matmul(H, P_pred, S, 2, 2, 2);
  S[0][0] += R[0][0]; S[0][1] += R[0][1];
  S[1][0] += R[1][0]; S[1][1] += R[1][1];
  
  float S_inv[2][2];
  float det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0];
  S_inv[0][0] = S[1][1] / det_S;
  S_inv[0][1] = -S[0][1] / det_S;
  S_inv[1][0]= -S[1][0] / det_S;
  S_inv[1][1] = S[0][0] / det_S;

  float K[2][2];
  matmul(P_pred, H, K, 2, 2, 2);
  matmul(K, S_inv, K, 2, 2, 2);

  // Actualizar el estado y la matriz de covarianza
  x[0] = x_pred[0] + K[0][0] * y[0] + K[0][1] * y[1];
  x[1] = x_pred[1] + K[1][0] * y[0] + K[1][1] * y[1];

  float I[2][2] = {{1, 0}, {0, 1}};
  float KH[2][2] = {{I[0][0] - K[0][0] * H[0][0] - K[0][1] * H[1][0], I[0][1] - K[0][0] * H[0][1] - K[0][1] * H[1][1]},
                    {I[1][0] - K[1][0] * H[0][0] - K[1][1] * H[1][0], I[1][1] - K[1][0] * H[0][1] - K[1][1] * H[1][1]}};

  matmul(KH, P_pred, P, 2, 2, 2);
}