#include <Adafruit_TCS34725.h>
#include <DynamixeliR.h>
#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>
#include <Wire.h>

//PORT ve PIN
#define DYNAMIXEL_PORT 3
#define INTAKE 5
#define frontHolder 9
#define frontDoor 10
#define backHolder 8
#define yanSolGoz 8
#define yanSagGoz 9
#define onSolGoz 10
#define onSagGoz 11

int driveTrainIndex[2][2] = {{3, 4}, {1, 2}}; // sol, sag (ön, arka)
int leftMotorsIndex[2] = {driveTrainIndex[0][0], driveTrainIndex[0][1]}; 
int rightMotorsIndex[2] = {driveTrainIndex[1][0], driveTrainIndex[1][1]};
int bagajIndex[3] = {7, 6, 11};  //red, green, blue (11, 7, 6)
int redBagajIndex = 0;
const int greenBagajIndex = 1;
int blueBagajIndex = 2;

//BAGGAGES
bool rb = false; // - tersi + saat yönü
bool bb = false; // - saat yönü + tersi
bool gb = false;
float rb7_90 = 530.0;
float bb11_90 = 512.0; // !!! NO INFO
const float gb6_90 = 512.0;
float rb7_180 = 840.0;
float bb11_180 = 200; // !!! NO INFO
const float gb6_180 = 200; 
//const float startFazladanAci = 188;

//SPEEDS
const int forwardLeft = 500;
const int forwardRight = forwardLeft + 1023;
const int backwardsLeft = 1524;
const int backwardsRight = backwardsLeft - 1023;
const int intakeSpeed = 500 + 1023;
const double wheelBase = 27.675 / 2;
//bool odometry = true;

softHalfDuplexSerial port(DYNAMIXEL_PORT);
dxliR dxlCom(&port); 

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

//GLOBAL VARIABLES

//TURN
const int keskinlik = 2;

//TIMING
unsigned int eskiZaman;
unsigned int yeniZaman;

//SENSOR VALS

//DISTANCE
int onSol;
int onSag;
int yanSol;
int yanSag;

//COLOR
uint16_t r, b, g, c;
float sum, red, green, blue;
float colorTolerance = 0.03;

//DOOR
bool doorClosed;

//HOLDER
bool isFrontHolderClosed, isFrontHolderLeft, isFrontHolderRight;
bool isBackHolderClosed, isBackHolderLeft, isBackHolderRight;

//int state;
String startingColor;
String currentColor;





void setup() {
  dxlCom.begin(57600);
  Serial.begin(9600);
  startBagaj();
  frontHolderClosed();
  backHolderClosed();
  frontDoorOpen();
  
  
  if (tcs.begin()) {
    Serial.print("OK");
  }
  else {
    Serial.println("Problem");
    while(1) {}
  }

  startingColor = color();
  Serial.println(startingColor);
  if (startingColor == "red") {
    Serial.print("kirmizi robot");
    redBagajIndex = 0;
    blueBagajIndex = 2;
    Serial.print(bagajIndex[redBagajIndex]);
    

    rb7_90 = rb7_90;
    bb11_90 = bb11_90;

    rb7_180 = rb7_180;
    bb11_180 = bb11_180;
  }
  else if (startingColor == "blue") { //blue
    Serial.print("mavi robot");
    forwardTimed(1000);
    
    blueBagajIndex = 0;
    redBagajIndex = 2;

    rb7_90 = bb11_90;
    bb11_90 = rb7_90;

    rb7_180 = bb11_180;
    bb11_180 = rb7_180;
  }

  redBagaj();
  greenBagaj();
  blueBagaj();

  pinMode(yanSolGoz, INPUT);
  pinMode(yanSagGoz, INPUT);
  pinMode(onSolGoz, INPUT);
  pinMode(onSagGoz, INPUT);

  torkOn();
  frontDoorOpen();
  //dxlCom.setGoalPosition(7, 500);
  //dxlCom.setMovingSpeed(1, 1000);
  //dxlCom.setMovingSpeed(2, 1000);
  //dxlCom.setMovingSpeed(3, 1000);
  //dxlCom.setMovingSpeed(4, 1000);
  //dxlCom.setMovingSpeed(11, 0);
  //Serial.println(dxlCom.readPresentPosition(6));
  /*
  dxlCom.setGoalPosition(7, 550.0);
  delay(1000);
  dxlCom.setGoalPosition(7, 840);
  delay(1000);
  dxlCom.setGoalPosition(7, 300.0);
  */

  //forward();
  //dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight);
  //dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight);
  //dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft);
  //dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft);

  frontDoorClosed();
  Serial.print(startingColor);

  if (startingColor == "red") {
    redBagaj();
    frontHolderRight();

  }

  else {
    redBagaj();
    frontHolderLeft();
    backHolderLeft();
  }
  

  delay(5000);
  
}

void loop() {
  forward();
 // distanceRead();


  /*
  Serial.print("onSol:"); Serial.print(onSol); Serial.print("\t");
  Serial.print("onSag:"); Serial.print(onSag); Serial.print("\t");
  Serial.print("yanSol:"); Serial.print(yanSol); Serial.print("\t");
  Serial.print("yanSag:"); Serial.print(yanSag); Serial.println("\t");*/

  /*
  if(onSol == HIGH) {
    unsigned int starting_time = millis();

    while(onSol == HIGH) {
      backwards();
      distanceRead();
    }

    stop();

    unsigned int ending_time = millis();
    unsigned int fark = ending_time - starting_time;
    Serial.println(fark);
  }*/
/*
  if(onSol == HIGH) {
    backwardsTimed(200);

    while(onSol == HIGH) {
      turnRightTimed(keskinlik, 200);
      distanceRead();
    }

    stop();
  }*/
  
  /*if (onSag == HIGH) {
    backwardsTimed(200);
    while(onSag == HIGH) {
      turnLeftTimed(keskinlik, 200);
      distanceRead();
    }

    stop();
  }*/

  /*else if (yanSol == HIGH) {
    while (yanSol == HIGH) {
      turnRightTimed(keskinlik, 200);
      distanceRead();
    }
    stop();
  }*/

  currentColor = color();

  Serial.print("RED: "); Serial.print(red);
  Serial.print("\tGREEN: "); Serial.print(green);
  Serial.print("\tBLUE: "); Serial.println(blue);

  if (currentColor == "red" && c >= 1000) {
    Serial.println("REDDDDD PUL");
    if (!doorClosed) frontDoorClosed();
    if (startingColor == "red") {
      if (rb) redBagaj();
      if (!isFrontHolderRight) frontHolderRight();
    }
    else {
      if (rb) redBagaj();
      if (!isBackHolderLeft) backHolderLeft();
      if (!isFrontHolderLeft) frontHolderLeft();
    }
  }

  else if (currentColor == "green" && c >= 1000) {
    Serial.println("GREEEENNN PUL");
    if(!doorClosed) frontDoorClosed();
    
  }

  else if (currentColor == "blue" && c >= 1000) {
    Serial.println("BLUEEEEEE PUL");
    if (!doorClosed) frontDoorClosed();
  } 

  else if (currentColor == "red") {
    Serial.println("Kırmızı Yer");
  }

  else if (currentColor == "green") {
    Serial.println("Yeşil yer");
  }

  else if (currentColor == "blue") {
    Serial.println("Mavi yer");
  }

  else {
    Serial.println("Either white or non");
    if (doorClosed) frontDoorOpen();

  }

}

//TORK
void torkOn() {
  dxlCom.setTorqueEnable(1, 1);
  
  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(2, 1);
  
  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(3, 1);
  
  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(4, 1);
  
  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(INTAKE, 1);
  
  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(6, 1);
  
  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(8, 1);
  
  while (!dxlCom.dxlDataReady())
    ;  

  dxlCom.setTorqueEnable(9, 1);
  
  while (!dxlCom.dxlDataReady())
    ;  

  dxlCom.setTorqueEnable(10, 1);
  
  while (!dxlCom.dxlDataReady())
    ;  

  dxlCom.setTorqueEnable(11, 1);
  
  while (!dxlCom.dxlDataReady())
    ;
    
}

//DRIVETRAIN
void forward() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft);
}

void forwardTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    forward();
    yeniZaman = millis();
  }
  stop();
}

void backwards() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
}

void backwardsTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;
  while (yeniZaman - eskiZaman <= time) {
    backwards();
    yeniZaman = millis();
  }
  stop();
}

void turnLeft(double fark_carpan) {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 500 * fark_carpan);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 500 * fark_carpan);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 500 * fark_carpan);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 500 * fark_carpan);
}

void turnLeftTimed(double fark_carpan, int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    turnLeft(fark_carpan);
    yeniZaman = millis();
  }

  stop();
  
}

void turnRight(double fark_carpan) {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 500 * fark_carpan);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 500 * fark_carpan);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 500 * fark_carpan);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 500 * fark_carpan);
}

void turnRightTimed(double fark_carpan, int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    turnRight(fark_carpan);
    yeniZaman = millis();
  }
  stop();
}

void stop() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 0);
}

//INTAKE
void intakeOn() {
  dxlCom.setMovingSpeed(INTAKE, intakeSpeed);

}

void intakeOff() {
  dxlCom.setMovingSpeed(INTAKE, 0);
}
c:\Users\sukru\OneDrive\Desktop\pul2025\nonavxpuls\libraries\SharpIR-master\examples\SharpSensorCm\SharpSensorCm.ino
//BAGGAGE
void redBagaj() {
  if (rb == false) {
    dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb7_90);
    rb = true;
  }

  else {
    dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb7_180);
    rb = false;
  }

}

void blueBagaj() {
  if (bb == false) {
    dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_90);
    bb = true;
  }
  else {
    dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_180);
    bb = false;
  }
}

void greenBagaj() {
  if (gb == false) {
    dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb6_90);
    gb = true;
  }
  else {
    dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb6_180);
    gb = false;
  }
}

void startBagaj() {
  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], 300);
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], 700);
  //dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_90 + startFazladanAci);
}

//HOLDER
void frontHolderClosed() {
  dxlCom.setGoalPosition(9, 495);
  while (!dxlCom.dxlDataReady());
  isFrontHolderClosed = true;
  isFrontHolderLeft = false;
  isFrontHolderRight = false;

}
void frontHolderRight() {
  dxlCom.setGoalPosition(9, 315);
  while (!dxlCom.dxlDataReady());
  isFrontHolderClosed = false;
  isFrontHolderLeft = false;
  isFrontHolderRight = true;

}
void frontHolderLeft() {
  dxlCom.setGoalPosition(9, 645);
  while (!dxlCom.dxlDataReady());
  isFrontHolderClosed = false;
  isFrontHolderLeft = true;
  isFrontHolderRight = false;

}

void backHolderRight() {
  dxlCom.setGoalPosition(backHolder, 340);
  while (!dxlCom.dxlDataReady());
  isBackHolderClosed = false;
  isBackHolderLeft = false;
  isBackHolderRight = true;

}
void backHolderClosed() {
  dxlCom.setGoalPosition(backHolder, 500);
  while (!dxlCom.dxlDataReady());
  isBackHolderClosed = true;
  isBackHolderLeft = false;
  isBackHolderRight = false;

}
void backHolderLeft() {
  dxlCom.setGoalPosition(backHolder, 690);
  while (!dxlCom.dxlDataReady());
  isBackHolderClosed = false;
  isBackHolderLeft = true;
  isBackHolderRight = false;

}

//DOOR
void frontDoorClosed() {
  dxlCom.setGoalPosition(frontDoor, 270);
  while (!dxlCom.dxlDataReady());
  doorClosed = true;

}
void frontDoorOpen() {
  dxlCom.setGoalPosition(frontDoor, 420);
  while (!dxlCom.dxlDataReady());
  doorClosed = false;

}

//DISTANCE
void distanceRead() {
  onSol = digitalRead(onSolGoz);
  onSag = digitalRead(onSagGoz);
  /*
  if (digitalRead(onSagGoz) == HIGH) {
    onSag = LOW;
  }
  else {
    onSag = HIGH;
  }*/

  if (digitalRead(yanSagGoz) == HIGH) {
    yanSag = LOW;
  }
  else {
    yanSag = HIGH;
  }

  if (digitalRead(yanSolGoz) == HIGH) {
    yanSol = LOW;
  }
  else {
    yanSol = HIGH;
  }
}

void wander() {
  distanceRead();
  getColors();

  
  if(onSol == HIGH) {
    backwardsTimed(200);

    while(onSol == HIGH) {
      turnRightTimed(keskinlik, 200);
      distanceRead();
    }

    stop();
  }
  
  else if (onSag == HIGH) {
    backwardsTimed(200);
    while(onSag == HIGH) {
      turnLeftTimed(keskinlik, 200);
      distanceRead();
    }

    stop();
  }

  else if (yanSol == HIGH) {
    while(yanSol == HIGH) {
      turnRightTimed(keskinlik, 200);
      distanceRead();
    }

    stop();
  }

  else if (yanSag == HIGH) {
    while(yanSag == HIGH) {
      turnLeftTimed(keskinlik, 200);
      distanceRead();
    }

    stop();
  }

  else {
    forward();
  }

  if ((red - colorTolerance) > green && (red - colorTolerance) > blue) {
    //red code
  }

  else if ((green - colorTolerance) > red && (green - colorTolerance) > blue) {
    //green code
  }

  else if ((blue - colorTolerance) > red && (blue - colorTolerance) > green) {
    //blue code
  }



  /*
  if(yanSol == HIGH) {
    while(yanSol == HIGH) {
      turnRight(keskinlik);
      distanceRead();
    }
  }

  else if (yanSag == HIGH) {
    while(yanSag == HIGH) {
      turnLeft(keskinlik);
      distanceRead();
    }
  }

  else if (onSol == HIGH) {
    backwardsTimed(500);
    while(onSol == HIGH) {
      turnRight(keskinlik);
      distanceRead();
    }
  }

  else if (onSag == HIGH) {
    backwardsTimed(500);
    while (onSag == HIGH) {
      turnLeft(keskinlik);
      distanceRead();
    }
  }
  */
}

//COLOR
void getColors() {
  tcs.getRawData(&r, &g, &b, &c);

  if (c>50) {
    sum = c;
    red = r / sum;
    green = g / sum;
    blue = b / sum;
  }
}

String color() {
  getColors();
  if ((red - colorTolerance) > green && (red - colorTolerance) > blue) {
    return "red";
  }

  else if ((green - colorTolerance) > red && (green - colorTolerance) > blue) {
    return "green";
  }

  else if ((blue - colorTolerance) > red && (blue - colorTolerance) > green) {
    return "blue";
  }

  else {
    return "";
  }
}



/*void turn(double degree) {
  if (odometry) {
    int difference = wheel_base * degree;

  }
}*/