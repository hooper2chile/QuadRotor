//probando subir a git
#include "library.h"

void setup()
{
  Wire.begin();
//Wire.setClock(1000000L);
  Wire.setClock(800000L);


  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);

//debug delay in functions i2c
  pinMode(A4, OUTPUT);
  digitalWrite(A4,LOW);

  // PID setting
  roll_pid.SetSampleTime(15);
  roll_pid.SetOutputLimits(-255,+255);

  input = afilter[1];
  setpoint = 0;// poner acá angulo variable
  roll_pid.SetMode(AUTOMATIC);

}



void loop()
{
  //etime = micros();
  // ---  reading IMU and data ---
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  raw_values();
  // --- end reading

  //Axis control
  x_axis();
  //y_axis();
  //z_axis();
  //etime = micros() - etime;
  //info();
  //delayMicroseconds(dt);

  return;
}








/*
//probando subir a git
#include "library.h"

void setup()
{
  Wire.begin();
//Wire.setClock(1000000L);
  Wire.setClock(800000L);


  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);

//debug delay in functions i2c
  pinMode(A4, OUTPUT);
  digitalWrite(A4,LOW);

  // PID setting
  roll_pid.SetSampleTime(15);
  roll_pid.SetOutputLimits(-255,+255);

  input = afilter[1];
  setpoint = 0;// poner acá angulo variable
  roll_pid.SetMode(AUTOMATIC);

}



void loop()
{
  //etime = micros();
  // ---  reading IMU and data ---
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  raw_values();
  // --- end reading

  //Axis control
  x_axis();
  //y_axis();
  //z_axis();
  //etime = micros() - etime;
  //info();
  //delayMicroseconds(dt);

  return;
}
*/