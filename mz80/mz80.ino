#include <Wire.h>
#include <MPU6050_light.h>
#include <DynamixeliR.h>
#include <SoftHalfDuplexSerial.h>
#include <SharpIR.h>
#include <Servo.h>
#include <CustomQTRSensors.h>

int inputPin = 9;
int inputPin2 = 5;
int inputPin3 = 2;
int inputPin4=10;
int onSolazer=0;
int val = 0;
int val2=0;
int val3=0;
softHalfDuplexSerial port(3);
dxliR dxlCom(&port);
void setup()
{
  Serial.begin(9600);
  dxlCom.begin(57600);
  pinMode(inputPin, INPUT);
    pinMode(inputPin2, INPUT);
     pinMode(inputPin3, INPUT);
       pinMode(inputPin4, INPUT);

       TorkOn();
}

void loop(){
  //dxlCom.setMovingSpeed(5, 600);

  /*
  dxlCom.setGoalPosition(14, 500);
  while (!dxlCom.dxlDataReady());
  //delay(250);
  dxlCom.setGoalPosition(14, 1000);
  while (!dxlCom.dxlDataReady());*/

  val = digitalRead(inputPin);//solmz
  Serial.println(inputPin);
  val2 = digitalRead(inputPin2); //sagon
  Serial.println(inputPin2);
  val3 = digitalRead(inputPin3);
  Serial.println(inputPin3);
   onSolazer = digitalRead(5);
  Serial.println(inputPin4);
  if(onSolazer==1){
//acik();   
 Serial.println("engel görüyor.");
 intake();
  }
  else{
//kapali();
intakeOff();
  }
  // if (val == HIGH) {eeeeeeeeeeeeeeeeeeerrrrrrrrrrrrrrrrrr
  //   digitalWrite(ledPin, LOW);
  //   Serial.println("Cisim yok!");
  // } else {
  //   digitalWrite(ledPin, HIGH);
  //   Serial.println("Cisim var!");
  // }
}
void acik() {
  dxlCom.setGoalPosition(14, 0);
  while (!dxlCom.dxlDataReady());

}
void kapali() {
  dxlCom.setGoalPosition(14, 200);
  while (!dxlCom.dxlDataReady());

}

void intake() {
  dxlCom.setMovingSpeed(5, 600);
  //while (!dxlCom.dxlDataReady());

}

void intakeOff() {
  dxlCom.setMovingSpeed(5, 0);
}

void TorkOn() {
 

  dxlCom.setTorqueEnable(14, 1); 
  while (!dxlCom.dxlDataReady());

  dxlCom.setTorqueEnable(5, 1); // intake
  while (!dxlCom.dxlDataReady());
}