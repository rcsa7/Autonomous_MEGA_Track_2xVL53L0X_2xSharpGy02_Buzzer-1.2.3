/*  ******************************************************************************************
 *    Copyright (c) 2021 by Roberto Carlos <robertocarlosalvarenga@gmail.com>
 *    Version    : 4.50
 *    Date       : 14/01/2021
 *   
 *    Description  : Implementa plataforma motora (Bob-Protótipo) com 2 Sensores VL53L0X (Frontal e Traseiro)
 *              
 *    References:
 *    https://www.arduino.cc/en/reference/SPI
 *    https://autocorerobotica.blog.br/conhecendo-o-protocolo-spi-com-arduino/
 *    https://github.com/pololu/vl53l0x-arduino
 *   
 *    Libraries:
 *    SPI.h
 *    Wire.h
 *    VL53L0X.h
 *   
 *    Libraries Source Code Paths:
 *    https://github.com/pololu/vl53l0x-arduino *    
 *    
 *    Arduino Preferences | Sketchbook Location:
 *    ... (Roberto)
 *    C:\Arduino\Sketchs\libraries (Jean)
 *    
 *    This file is free software; you can redistribute it and/or modify
 *    it under the terms of either the GNU General Public License version 2
 *    or the GNU Lesser General Public License version 2.1, both as
 *    published by the Free Software Foundation.
 */

#include <SharpIR_Roberto.h>  // biblioteca específica para o sensor GY21SHARP
#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>

/* Model :
   GP2Y0A21YK0F --> 1080
 
*/
#define SHARP_MODEL 20150

VL53L0X sensorFrontal;
VL53L0X sensorTraseiro;
#define LONG_RANGE

//declar OLED pins
const int SDA_PIN = 20;
const int SDC_PIN = 21;

const byte driverSensorPin = A0; // sensor lateral--esquerdo--MOTORISTA
const byte passengerSensorPin = A1;// sensor lateral DIREITO--passageiro
SharpIR_Roberto driverSensor = SharpIR_Roberto(driverSensorPin, SHARP_MODEL);
SharpIR_Roberto passengerSensor = SharpIR_Roberto(passengerSensorPin, SHARP_MODEL);

#define PIN1 22 // Pino do SensorFrontal
#define PIN2 23 // Pino do SensorTraseiro
#define LED1 7
#define LED2 8
#define BUZZER_PIN  49   // AX-1205-2 5V


// connect motor controller pins to Arduino digital pins
// motor one
int motorA = 3; //pin-pwm-a-- 
int in1 = 12;//--dir-a---in1
int in2 = 13;//--dir-b---in2

// motor two
int motorB = 11; //pin-pwm- b--
int in3 = 12; // dir--b--in3
int in4 =  13; // dir--a--in4

void setup()
{  
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //--------------------------------------
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  tone(BUZZER_PIN, 200, 200); delay(200); 
  tone(BUZZER_PIN, 500, 400); delay(500); 
Wire.begin();
Serial.begin (115200);

pinMode(PIN1, OUTPUT);
pinMode(PIN2, OUTPUT);
digitalWrite(PIN1, LOW);
digitalWrite(PIN2, LOW);

pinMode(BUZZER_PIN, OUTPUT); 
tone(BUZZER_PIN, 200, 200); delay(200); 
tone(BUZZER_PIN, 500, 400); delay(500);  
delay(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensorFrontal.setSignalRateLimit(0.1);
  sensorTraseiro.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensorFrontal.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);//default
  sensorFrontal.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);//default
  //---------
 sensorTraseiro.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);//default
  sensorTraseiro.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);//default
  
#endif




pinMode(PIN1, INPUT);
delay(150);
Serial.println("00");
sensorFrontal.init(true);

Serial.println("01");
delay(100);
sensorFrontal.setAddress((uint8_t)23);
Serial.println("02");

pinMode(PIN2, INPUT);
delay(150);
sensorTraseiro.init(true);
Serial.println("03");
delay(100);
sensorTraseiro.setAddress((uint8_t)25);
Serial.println("04");

Serial.println("addresses set");

// end configuration
sensorFrontal.setTimeout(500);
sensorTraseiro.setTimeout(500);
sensorFrontal.startContinuous();
sensorTraseiro.startContinuous();

// scan i2c
Serial.println ("I2C scanner. Scanning ...");
byte count = 0;

for (byte i = 1; i < 120; i++) {

Wire.beginTransmission (i);
if (Wire.endTransmission () == 0) {
Serial.print ("Found address: ");
Serial.print (i, DEC);
Serial.print (" (0x");
Serial.print (i, HEX);
Serial.println (")");
count++;
delay (1); // maybe unneeded?
} // end of good response
} // end of for loop
Serial.println ("Done.");
Serial.print ("Found ");
Serial.print (count, DEC);
Serial.println (" device(s).");
  //---------------------------------------------------------
  callI2Cscanner();
}

void callI2Cscanner()
{
  // scan i2c
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++) 
  {  
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    }             // end of good response
  }               // end of for loop
  
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");  
}
void demoOne()
{  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // ajuste pwm range 0~255
  analogWrite(motorA, 200);
  // liga o motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
 // ajuste pwm range 0~255
  analogWrite(motorB, 150);
  delay(2000);
  // direcao motores
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  delay(2000);
  // desliga motores
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void forward()
{
  digitalWrite(in1, LOW); //IN1
  digitalWrite(in2, HIGH);//IN2
  analogWrite(motorA, 150);
  digitalWrite(in3, HIGH);//IN3
  digitalWrite(in4, LOW);  //IN4
  analogWrite(motorB, 150);
}

void runback ()
{
  digitalWrite(in1, LOW); //IN1
  digitalWrite(in2, HIGH);//IN2
  analogWrite(motorA, 150);
  digitalWrite(in3, LOW);//IN3
  digitalWrite(in4, HIGH);  //IN4
  analogWrite(motorB, 150);
}

 //----para direita
void turnRight(){
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
analogWrite(motorA, 0);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW); 
analogWrite(motorB, 200); 
}

//---para esquerda
void turnLeft(){
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
analogWrite(motorA, 200);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
analogWrite(motorB, 0); 
}
void Stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop()
{
   //-------------------------------------------------
  
  //-vl53l0x----sensorFrontal-----------
  Serial.print(sensorFrontal.readRangeContinuousMillimeters());
  if (sensorFrontal.timeoutOccurred()) { Serial.print(" TIMEOUT S1"); }
  Serial.print("\t");
  Serial.println();
  float distanciaFrontal= sensorFrontal.readRangeContinuousMillimeters();
  //-------------------------------------------------------------------
  //-----sensorTraseiro
  Serial.print(sensorTraseiro.readRangeContinuousMillimeters());
  if (sensorTraseiro.timeoutOccurred()) { Serial.print(" TIMEOUT S2"); }
  Serial.print("\t");
  Serial.println();
  float distanciaTraseira= sensorTraseiro.readRangeContinuousMillimeters();
  //----------------------------------------------------------------------
  
  //-----------sensores sharp-----------------------
  int driverDistance;
  int passengerDistance;
  driverDistance = driverSensor.distance();
  passengerDistance = passengerSensor.distance();

  //---sensor lado motorista
   if (driverDistance > 400 && driverDistance < 600)
  { 
  ///  digitalWrite(LED1, LOW); 
    forward();
  }
    //---sensor lado motorista//------//---------------------------
   if (driverDistance > 100 && driverDistance < 300)
  { 
    Stop();
    delay(300);
    runback(); 
    delay(400);
    Stop();
    delay(300);
    turnRight();
    Stop();
    delay(300);
   /// digitalWrite(LED1, LOW); 
  }
  else {
    forward();
  }

  //----sensor lado passageiro//-----//---------------------
     if (passengerDistance > 100 && passengerDistance < 300)
  { 
     
    Stop();
    delay(300);
    runback(); 
    delay(400);
    Stop();
    delay(300);
    turnLeft();
    Stop();
    delay(300);
   /// digitalWrite(LED1, LOW);
  }
   else 
   {
    forward();
  }

  
  //----sensor lado passageiro----------------------------------
     if (passengerDistance > 400 && passengerDistance < 600)
  { 
   /// digitalWrite(LED1, LOW); 
    forward();
  }
  
//-------SENSOR LV53-------------------------------------------------------------- 
 //---PARA FRENTE--
  if (distanciaFrontal > 200 && distanciaFrontal < 500)
  { 
    ///digitalWrite(LED1, LOW); 
    forward();
  }
//---PARA DIREITA
   if (distanciaFrontal > 600 && distanciaFrontal < 650)
  { 
    turnRight();
    delay(200);
   /// digitalWrite(LED1, LOW); 
    forward();
  }
  //---PARA ESQUERDA
    if (distanciaFrontal > 680 && distanciaFrontal < 750)
  { 
   turnLeft();
    delay(200);
   /// digitalWrite(LED1, LOW); 
    forward();
  }
  //----para traseira
  if (distanciaTraseira > 500 && distanciaTraseira < 650)
  { 
  ///  digitalWrite(LED1, LOW); 
    runback(); 
   tone(BUZZER_PIN, 300, 300); // 
  }
//----loop
   if (distanciaTraseira > 660 && distanciaTraseira < 800)
  { 
   /// digitalWrite(LED1, LOW); 
   demoOne(); 
   tone(BUZZER_PIN, 200, 200); // 
  }

  //----parada total  if (distanciaFrontal > 650 && distanciaFrontal < 750 || distanciaTraseira > 650 && distanciaTraseira < 750)
  {
      analogWrite(motorA, LOW);
      analogWrite(motorB, LOW);
      digitalWrite(LED1, HIGH);  
      digitalWrite(LED2, HIGH);  
  }
}
 
