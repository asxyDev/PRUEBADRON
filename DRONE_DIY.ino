// Dron ESP32 con MPU6050, Filtro de Kalman, PID y control de motores modularizado

#include <Wire.h>
#include <ESP32Servo.h>

// -------------------- Pines PWM Receptor --------------------
const int CH_PIN[6] = {34, 35, 32, 33, 25, 26};  // Roll, Pitch, Throttle, Yaw, Aux1, Aux2

// -------------------- Pines ESC/Motores --------------------
const int mot1_pin = 27;
const int mot2_pin = 13;
const int mot3_pin = 12;
const int mot4_pin = 4;

// -------------------- Variables PWM --------------------
volatile uint32_t chStart[6] = {0};
volatile int pulseWidth[6] = {0};

// -------------------- IMU raw data --------------------
int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngleRoll, accAnglePitch;
float gyroRateRoll, gyroRatePitch, gyroRateYaw;

// -------------------- Kalman filter --------------------
typedef struct { float angle; float uncertainty; } Kalman1D;
Kalman1D kalmanRoll = {0, 4};
Kalman1D kalmanPitch = {0, 4};
const float dt = 0.008; // 8 ms loop

// -------------------- PID constants --------------------
const float Kp = 1.3, Ki = 0.02, Kd = 0.4;
const float KpYaw = 1.5, KiYaw = 0.01, KdYaw = 0.3;

// -------------------- PID variables --------------------
float errRoll, prevErrRoll, intErrRoll;
float errPitch, prevErrPitch, intErrPitch;
float errYaw, prevErrYaw, intErrYaw;

// -------------------- Motor/Throttle safety -------------------
const int throttleIdle = 1170;
const int throttleCutoff = 1000;
const int ESCfreq = 400;

// Servos motores
Servo mot1, mot2, mot3, mot4;

unsigned long loopTimer;

// ---- ISRs lectura PWM receptor ----
void IRAM_ATTR ISR_CH1() { if (digitalRead(CH_PIN[0])) chStart[0] = micros(); else pulseWidth[0] = micros() - chStart[0]; }
void IRAM_ATTR ISR_CH2() { if (digitalRead(CH_PIN[1])) chStart[1] = micros(); else pulseWidth[1] = micros() - chStart[1]; }
void IRAM_ATTR ISR_CH3() { if (digitalRead(CH_PIN[2])) chStart[2] = micros(); else pulseWidth[2] = micros() - chStart[2]; }
void IRAM_ATTR ISR_CH4() { if (digitalRead(CH_PIN[3])) chStart[3] = micros(); else pulseWidth[3] = micros() - chStart[3]; }
void IRAM_ATTR ISR_CH5() { if (digitalRead(CH_PIN[4])) chStart[4] = micros(); else pulseWidth[4] = micros() - chStart[4]; }
void IRAM_ATTR ISR_CH6() { if (digitalRead(CH_PIN[5])) chStart[5] = micros(); else pulseWidth[5] = micros() - chStart[5]; }

// ---- Filtro de Kalman 1D ----
Kalman1D kalman1d(Kalman1D state, float measAngle, float measRate, float dt) {
  state.angle += dt * measRate;
  state.uncertainty += dt * dt * 16; // varianza giroscopio
  float K = state.uncertainty / (state.uncertainty + 9); // varianza acelerómetro
  state.angle += K * (measAngle - state.angle);
  state.uncertainty *= (1 - K);
  return state;
}

// ---- Lectura MPU6050 ----
void updateIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (size_t)14, true);

  AccX = Wire.read()<<8 | Wire.read();
  AccY = Wire.read()<<8 | Wire.read();
  AccZ = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read();
  GyroX = Wire.read()<<8 | Wire.read();
  GyroY = Wire.read()<<8 | Wire.read();
  GyroZ = Wire.read()<<8 | Wire.read();

  accAngleRoll  = atan2(AccY, AccZ) * RAD_TO_DEG;
  accAnglePitch = atan(-AccX / sqrt(AccY*AccY + AccZ*AccZ)) * RAD_TO_DEG;
  gyroRateRoll  = GyroX / 131.0;
  gyroRatePitch = GyroY / 131.0;
  gyroRateYaw   = GyroZ / 131.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Inicializar MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission();

  // Configurar pines PWM receptor e ISRs
  for (int i = 0; i < 6; i++) {
    pinMode(CH_PIN[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(CH_PIN[0]), ISR_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PIN[1]), ISR_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PIN[2]), ISR_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PIN[3]), ISR_CH4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PIN[4]), ISR_CH5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_PIN[5]), ISR_CH6, CHANGE);

  // Configurar motores
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);
  mot1.setPeriodHertz(ESCfreq);
  mot2.setPeriodHertz(ESCfreq);
  mot3.setPeriodHertz(ESCfreq);
  mot4.setPeriodHertz(ESCfreq);
  mot1.writeMicroseconds(throttleIdle);
  mot2.writeMicroseconds(throttleIdle);
  mot3.writeMicroseconds(throttleIdle);
  mot4.writeMicroseconds(throttleIdle);

  loopTimer = micros();
}

void loop() {
  updateIMU();

  // Filtrado Kalman
  kalmanRoll  = kalman1d(kalmanRoll, accAngleRoll,  gyroRateRoll,  dt);
  kalmanPitch = kalman1d(kalmanPitch, accAnglePitch, gyroRatePitch, dt);

  // PID Roll
  errRoll = (pulseWidth[0] - 1500) * 0.1 - kalmanRoll.angle;
  intErrRoll += errRoll * dt;
  float derRoll = (errRoll - prevErrRoll) / dt;
  float outRoll = Kp * errRoll + Ki * intErrRoll + Kd * derRoll;
  prevErrRoll = errRoll;

  // PID Pitch
  errPitch = (pulseWidth[1] - 1500) * 0.1 - kalmanPitch.angle;
  intErrPitch += errPitch * dt;
  float derPitch = (errPitch - prevErrPitch) / dt;
  float outPitch = Kp * errPitch + Ki * intErrPitch + Kd * derPitch;
  prevErrPitch = errPitch;

  // PID Yaw (solo giroscopio)
  errYaw = (pulseWidth[3] - 1500) * 0.1 - gyroRateYaw;
  intErrYaw += errYaw * dt;
  float derYaw = (errYaw - prevErrYaw) / dt;
  float outYaw = KpYaw * errYaw + KiYaw * intErrYaw + KdYaw * derYaw;
  prevErrYaw = errYaw;

  // Throttle y seguridad
  int throttle = pulseWidth[2];
  if (throttle < throttleCutoff) {
    mot1.writeMicroseconds(throttleCutoff);
    mot2.writeMicroseconds(throttleCutoff);
    mot3.writeMicroseconds(throttleCutoff);
    mot4.writeMicroseconds(throttleCutoff);
    intErrRoll = intErrPitch = intErrYaw = 0;
    prevErrRoll = prevErrPitch = prevErrYaw = 0;
  } else {
    throttle = min(throttle, 1800);
    int m1 = throttle - outRoll - outPitch - outYaw;
    int m2 = throttle - outRoll + outPitch + outYaw;
    int m3 = throttle + outRoll + outPitch - outYaw;
    int m4 = throttle + outRoll - outPitch + outYaw;
    m1 = constrain(m1, throttleIdle, 2000);
    m2 = constrain(m2, throttleIdle, 2000);
    m3 = constrain(m3, throttleIdle, 2000);
    m4 = constrain(m4, throttleIdle, 2000);
    mot1.writeMicroseconds(m1);
    mot2.writeMicroseconds(m2);
    mot3.writeMicroseconds(m3);
    mot4.writeMicroseconds(m4);
  }

  // VALORES IMU
  Serial.print("AccX: "); Serial.print(AccX);
  Serial.print(" | AccY: "); Serial.print(AccY);
  Serial.print(" | AccZ: "); Serial.print(AccZ);
  Serial.print(" || GyroX: "); Serial.print(GyroX);
  Serial.print(" | GyroY: "); Serial.print(GyroY);
  Serial.print(" | GyroZ: "); Serial.print(GyroZ);
  Serial.print(" || AccRoll: "); Serial.print(accAngleRoll);
  Serial.print(" | AccPitch: "); Serial.print(accAnglePitch);
  Serial.print(" | GyroYaw: "); Serial.println(gyroRateYaw);

  // Control de tiempo de muestreo
  while (micros() - loopTimer < dt * 1e6);
  loopTimer = micros();
}
