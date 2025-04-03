/*
 * Código para MK INDUSTRIAL
 */

#include <Arduino.h>
#include <QTRSensors.h>

// Definicion de pines
#define bot 4
#define led 13
#define PWMM1B 11 //Para adelante
#define PWMM1A 12 //Para atras
#define ENM1 7
#define ENM2 8
#define PWMM2A 9 //Para atras
#define PWMM2B 6 //Para adelante

// Constantes de sensado y posicion
#define sensorCount 6
#define errorCount 3 // Cantidad de errores para calcular promedio y eliminar ruido
#define centroLinea 2500
#define maxOffset 1500

// Constantes de los motores y velocidades
#define velBase 100  // Valor base de velocidad del motor
#define velMin 0    // Valor minimo de velocidad
#define velMax 255  // Valor maximo de velocidad

// Variables de sensado y posicion
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
int position = 0;
int error = 0;
int offset = 0, targetOffset = 0;

// Velocidades
unsigned int vel = 120;  // Promedio de velocidad entre los dos motores
double motVel = 0;      // Salida del PID
double mAVel = vel;     // Velocidad motor A
double mBVel = vel;     // Velocidad motor B
unsigned int targetVel = 255;

// Variables globales adicionales para el control PID
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError = 0, rateError;
double lastError = 0;

// Parámetros del PID
double Kp = 0.07;
double Ki = 0.00001;
double Kd = 0.004;

// Variables para los botones
int prevState = 0;
int flag = 0;  // Si flag es 1, estan los dos motores al palo
unsigned long timePressed = 0;
unsigned long startedPressing = 0;
int ledState = 0;
unsigned long lastBlink = 0;

// Esta función se encarga de la configuración inicial del sistema
void setup() {
  configureIO();       // Configura entradas y salidas
  configureSensors();  // Configura los sensores
  configureMotor();    // Configura los motores

  //configurePID();
  configureEEPROM();

  calibration();

  //Serial.begin(9600);
  //printCalibration();

  while (!funBotonesSetup()) {}  // Si retorna 0 sigue esperando, si retorna 1 el auto empieza a andar

  previousTime = currentTime;
}

// Esta función se encarga de leer los sensores y controlar los motores
void loop() {
  funBotonesLoop();

  readSensors();
  lineOffset();
  getTimeData();

  controlVelocity();

  controlMotors();
  restrictMotorSpeed();
  applySpeed();
}

// Función para el control de los botones
int funBotonesSetup() {
  unsigned long actualTime = millis();
  int state = !digitalRead(bot);

  //Serial.println(state);

  if (state == 0 && prevState == 0)  // No se esta apretando el boton
  {
    if (ledState == 0 && actualTime - lastBlink > 182)  // En honor a blink-182
    {
      lastBlink = actualTime;
      ledState = 1;
      digitalWrite(led, HIGH);
    } else if (ledState == 1 && actualTime - lastBlink > 182) {
      lastBlink = actualTime;
      ledState = 0;
      digitalWrite(led, LOW);
    }
  } else if (state == 1 && prevState == 0)  // Se empezo a apretar el boton
  {
    prevState = 1;
    startedPressing = actualTime;
  } else if (state == 1 && prevState == 1)  // Se esta apretando el boton
  {
    timePressed = actualTime - startedPressing;

    if (timePressed < 2000) {
      digitalWrite(led, LOW);
    } else {
      digitalWrite(led, HIGH);
    }
  } else if (state == 0 && prevState == 1)  // Se dejo de apretar el boton
  {
    prevState = 0;
    timePressed = actualTime - startedPressing;
  }

  if (state == 0 && timePressed != 0)  // Si esta apretado seguir esperando
  {
    // Manejo de las flags
    if (timePressed < 2000 && flag == 0) {
      flag = 1;
      timePressed = 0;
      analogWrite(PWMM1B, 255);
      analogWrite(PWMM2B, 255);
    } else if (timePressed < 2000 && flag == 1) {
      flag = 0;
      timePressed = 0;
      analogWrite(PWMM1B, 0);
      analogWrite(PWMM2B, 0);
    } else if (timePressed >= 2000) {
      return 1;
    }
  }

  return 0;
}

void funBotonesLoop(){
  bool buttonState = digitalRead(bot), prevButtonState = false;
  if (buttonState){
    prevButtonState = true;
    mAVel = 0;
    mBVel = 0;
    digitalWrite(led, LOW);
    applySpeed();
    digitalWrite(led, HIGH);
    while (1){
      buttonState = digitalRead(bot);
      if (buttonState && !prevButtonState){
        return;
      }
    }
  }
}

// Configuración inicial de los sensores
void configureSensors() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A6, A3, A2, A1, A0 }, sensorCount);
}

// Configuración inicial del motor
void configureMotor() {
  digitalWrite(PWMM2A, LOW);
  digitalWrite(ENM1, HIGH);
  digitalWrite(PWMM1A, LOW);
  digitalWrite(ENM2, HIGH);
  analogWrite(PWMM1B, 0);
  analogWrite(PWMM2B, 0);
}

// Proceso de calibración de los sensores
void calibration() {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(led, LOW);
}

// Configura los pins de entrada/salida
void configureIO() {
  pinMode(led, OUTPUT);
  pinMode(bot, INPUT_PULLUP);
  pinMode(PWMM1A, OUTPUT);
  pinMode(PWMM2A, OUTPUT);
  pinMode(ENM1, OUTPUT);
  pinMode(ENM2, OUTPUT);
  pinMode(PWMM1B, OUTPUT);
  pinMode(PWMM2B, OUTPUT);
}


// Lee los valores de los sensores
void readSensors() {
  error = 0;
  for (int i = 0; i < errorCount; i++) {
    uint16_t position = qtr.readLineWhite(sensorValues);
    error += position - centroLinea + offset;
  }
  error /= errorCount;
}

void getTimeData() {
  currentTime = millis();  // Obtener el tiempo actual

  elapsedTime = (double)(currentTime - previousTime) / 1000;  // Calcular el tiempo transcurrido desde el cálculo anterior en segundos
}

// Controla los motores basándose en los valores de los sensores
void controlMotors() {
  cumError += ((error + lastError) / 2) * elapsedTime;  // Calcular la integral del error
  rateError = (error - lastError) / elapsedTime;        // Calcular la derivada del error

  motVel = Kp * error + Ki * cumError + Kd * rateError;  // Calcular la salida del PID

  lastError = error;           // Recordar el error actual
  previousTime = currentTime;  // Recordar el tiempo actual

  // Se calcula el PWM de cada motor en base a la salida del PID (motvel)
  mAVel = vel + motVel;
  mBVel = vel - motVel;
}

void applySpeed() {
  // Aplicar velocidades
  analogWrite(PWMM1B, mAVel);
  analogWrite(PWMM2B, mBVel);
}

// Restringe la velocidad del motor a valores seguros
void restrictMotorSpeed() {
  if (mAVel > velMax) {
    mBVel -= mAVel - velMax;
  } else if (mAVel < velMin) {
    mBVel += velMin - mAVel;
  }

  if (mBVel > velMax) {
    mAVel -= mBVel - velMax;
  } else if (mBVel < velMin) {
    mAVel += velMin - mBVel;
  }

  mAVel = constrain(mAVel, velMin, velMax);
  mBVel = constrain(mBVel, velMin, velMax);
}
