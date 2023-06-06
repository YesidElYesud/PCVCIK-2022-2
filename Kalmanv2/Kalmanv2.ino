#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Instanciar el objeto BNO055
Adafruit_BNO055 sensor = Adafruit_BNO055();

// Variables del filtro de Kalman
float dt = 0.01; // Intervalo de tiempo entre mediciones (segundos)
float A[2][2] = {{1, dt}, {0, 1}}; // Matriz de transición de estado (cambia el signo de -dt a dt)
float H[2][2] = {{1, 0}, {0, 1}}; // Matriz de observación (cambia el tamaño de 1x2 a 2x2)
float Q[2][2] = {{0.001, 0}, {0, 0.001}}; // Matriz de ruido del proceso
float R[2][2] = {{0.5, 0}, {0, 0.5}}; // Matriz de ruido de la medición (cambia el tamaño de 1x1 a 2x2 y agrega varianza para la velocidad angular)
float P[2][2] = {{1, 0}, {0, 1}}; // Matriz de covarianza inicial
float x[2] = {0, 0}; // Estado inicial (ángulo y velocidad angular)

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!sensor.begin()) {
    Serial.print("¡Error al iniciar el BNO055!");
    while (1);
  }
  delay(1000);
  sensor.setExtCrystalUse(true);
}

void loop() {
  // Leer datos del BNO055
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
  

  delay(10);
}