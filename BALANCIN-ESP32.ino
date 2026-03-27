U2-P1
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Librería para los motores brushless
#include <Servo.h>
Servo right_prop;
Servo left_prop;

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

int pwmIzquierda,pwmDerecha;
float PID,error, error_prev;
float pid_p=0;
float pid_i=0;
float pid_d=0;
///////////////// CONSTANTES PID/////////////////
double kp=3.0;//3.55
double ki=0.0;//0.003
double kd=0.0;//2.05
///////////////////////////////////////////////

unsigned long t1 = 0, t2 = 0; // Tiempos para intervalos
int dt_us = 600; // Ciclos de 600 microsegundos
double valorMin = 1300;
float angulo_deseado = 0; //El angulo en el que queremos que el 
                         //balancín se mantenga

void setup() {
  Serial.begin(250000);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  // Inicializamos los motores
  right_prop.attach(3); //attatch the right motor to pin 3
  left_prop.attach(6);  //attatch the left motor to pin 5
  right_prop.writeMicroseconds(valorMin);
  left_prop.writeMicroseconds(valorMin); 
  delay(3500); // Delay para que inicie todo



}

void loop() {

  t1 = micros();
  ///////////////////////////////////////// COMIENZA EL PERIODO DE CONTROL /////////////////////////////
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

  //Mostrar los angulos separadas por un [tab]
  /*
  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x); 
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);*/


  
  ////////////////////////////P I D////////////////////////////////////

  error = ang_y - angulo_deseado;

  pid_p = kp*error;

  if(error > -3 && error < 3)
    {
    pid_i = pid_i+(ki*error);  
    }

  pid_d = kd*((error - error_prev)/dt);

  //The final PID values is the sum of each of this 3 parts/
  PID = pid_p + pid_i + pid_d;

  if(PID < -1000)
  {
    PID=-1000;
  }
  if(PID > 1000)
  {
    PID=1000;
  }

  //Finnaly we calculate the PWM width. We sum the desired throttle and the PID value/
  pwmIzquierda = valorMin + PID;
  pwmDerecha = valorMin - PID;

  if(pwmDerecha < 1000)
{
  pwmDerecha= 1000;
}
if(pwmDerecha > 2000)
{
  pwmDerecha=2000;
}
//Left
if(pwmIzquierda < 1000)
{
  pwmIzquierda= 1000;
}
if(pwmIzquierda > 2000)
{
  pwmIzquierda=2000;
}

left_prop.writeMicroseconds(pwmIzquierda);
right_prop.writeMicroseconds(pwmDerecha);
error_prev = error; 



  ///////////////// GRAFICA ANGULO DESEADO Y ANGULO ACTUAL ///////////////
  /*
  Serial.print(angulo_deseado);
  Serial.print(" ");
  Serial.println(ang_y);
 */
  ///////////////// GRAFICA ERROR Y SEÑALES DE CONTROL ///////////////
  
  Serial.print(error);
  Serial.print(" ");
  Serial.print(pwmIzquierda);
  Serial.print(" ");
  Serial.println(pwmDerecha);

  /////////////////////////// TERMINA PERIODO DE CONTROL /////////////////////
 /* t2 = micros();
  while((t2-t1)<dt_us){
    t2 = micros();
  }
  Serial.print("Muestreo constante de: ");
  //Serial.print(t2-t1);
  Serial.println("microsegundos");*/
}