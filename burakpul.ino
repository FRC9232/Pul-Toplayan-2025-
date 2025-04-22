#include <Wire.h>
#include <DynamixeliR.h>
#include <SoftHalfDuplexSerial.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <SharpIR.h>

#define sag A0
#define sol A3
#define onSag A1
#define onSol A2
#define model 20150

// Her sensör için ayrı bir nesne oluştur
SharpIR ir_onSol(onSol, model);
SharpIR ir_onSag(onSag, model);
SharpIR ir_sol(sol, model);
SharpIR ir_sag(sag, model);


#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
float red, green, blue;
int r, g, b, c;
// our RGB -> eye-recognized gamma color
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int driveTrainIndex[2][2] = { { 3, 4 }, { 1, 2 } };  // sol, sag (ön, arka)
int leftMotorsIndex[2] = { driveTrainIndex[0][0], driveTrainIndex[0][1] };
int rightMotorsIndex[2] = { driveTrainIndex[1][0], driveTrainIndex[1][1] };
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground
// set to false if using a common cathode LED

//SPEEDS
const int forwardLeft = 350;
const int forwardRight = forwardLeft + 1023;
const int backwardsLeft = 1524;
const int backwardsRight = backwardsLeft - 1023;
const int intakeSpeed = 700 + 1023;
const double wheelBase = 27.675 / 2;
unsigned int eskiZaman;
unsigned int yeniZaman;

//-----Dynamixel------
softHalfDuplexSerial port(3);
dxliR dxlCom(&port);
//-----Dynamixel End--

int yesilkapaksayisi = 0;
int istenenkapaksayisi = 0;
int istenmeyenkapaksayisi = 0;
String startZemin = "blue";

#define backHolder 8
#define frontDoor 10

int bagajIndex[3] = { 14, 6, 16 };  //red, green, blue (11, 7, 6)
int redBagajIndex = 0;
const int greenBagajIndex = 1;
int blueBagajIndex = 2;

bool rb = false;  // - tersi + saat yönü
bool bb = false;  // - saat yönü + tersi
bool gb = false;
String color = "";
String zemin = "";

float rb7_90 = 530.0;
float bb11_90 = 400.0;  // !!! NO INFO
const float gb6_90 = 512.0;
float rb7_180 = 200.0;
float bb11_180 = 850.0;  // !!! NO INFO
const float gb6_180 = 200.0;

String state = "wallScan";

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1) {}
  }

  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }

  dxlCom.begin(57600);
  Wire.begin();
  //prepareGyro();
  torkOn();
  startBagaj();
  frontHolderClosed();
  backHolderClosed();
  frontDoorOpen();
  istenenBagajClosed();
  greenBagajClosed();
  istenmeyenBagajClosed();
}

void loop() {

  Serial.print("State : ");
  Serial.println(state);
mesafeOku();
int dis_onSol = ir_onSol.distance();
    int dis_onSag = ir_onSag.distance();
    int dis_sol = ir_sol.distance();
    int dis_sag = ir_sag.distance();
  if (state == "wallScan") {
    int dis_onSol = ir_onSol.distance();
    int dis_onSag = ir_onSag.distance();
    int dis_sol = ir_sol.distance();
    int dis_sag = ir_sag.distance();
    if (dis_sag < 25) {
      Serial.println("Sağımda engel görüyorum");
      Stop();
      //turnRightTimed(400);
      turnLeftDelayed();
    } else if (dis_sol < 25) {
      Serial.println("solumda engel görüyorum");
      Stop();
      // turnRightTimed(360);
      turnRightDelayed();
    } else if (dis_onSag < 34) {
      Serial.println("Ön solda engel görüyorum");
      Stop();
      //backwardsTimed(160);
      backwardsDelayed();
      Stop();
      //turnLeftTimed(400);
      turnLeftDelayed();
    } else if (dis_onSol < 34) {
      Serial.println("Ön sagda engel görüyorum");
      Stop();
      // backwardsTimed(160);
      backwardsDelayed();
      Stop();
      //turnRightTimed(400);
      turnRightDelayed();
    } else {
      forward();
      state = "collection";
    }
  }                                  //wallScan finish
  else if (state == "collection") {  //Kapak Toplama
    intake();
    renkOku();
    if (startZemin == "blue") {

      if ((red - green > 70) && (red - blue > 75) && c > 120) {
        color = "Red";
        Stop();
        //Serial.println("kırmızı");
        frontDoorClosed();
        frontHolderLeft();
        backHolderLeft();
        forwarddelay(310);
        istenmeyenkapaksayisi++;
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else if ((blue - green > 7) && (blue - red > 18) && c > 120) {
        color = "Blue";
        //Serial.println("mavi");
        Stop();
        frontDoorClosed();
        frontHolderRight();
        istenenkapaksayisi++;
        forwarddelay(310);
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else if ((green - blue > 20) && (green - red > 20) && c > 120) {
        color = "Green";
        //Serial.println("yesil");
        Stop();
        frontDoorClosed();
        frontHolderLeft();
        backHolderRight();
        forwarddelay(330);
        yesilkapaksayisi++;
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else {
        state = "wallScan";
      }

      /* ----- Zemin Görmesi Start ----- */
      if ((blue - green > 7) && (blue - red > 18) && c < 120 && istenenkapaksayisi > 0) {
        zemin = "Blue";
        Stop();
        Serial.println("Blue Zemin");
        forwarddelay(400);
        Stop();
        delay(300);
        istenenkapaksayisi = 0;
        // delay(2000);
        mesafeOku();
        if (dis_sag < dis_sol) {  // Zeminin içinde dönmesi
          // backwardsTimed(100);
          //backwardsDelayed();
          Stop();
          //  turnRightTimed(500);
          turnRightDelayed();
          istenenBagajOpen();
          forwarddelay(1000);
          istenenBagajClosed();
          Serial.println("Sağa dönüyorum");
        } else {
          backwardsDelayed();
          // backwardsTimed(100);
          Stop();
          // turnLeftTimed(500);
          turnLeftDelayed();
          istenenBagajOpen();
          Serial.println("Sola Dönüyorum");
          forwarddelay(1000);
          //bagajkapatılır
          istenenBagajClosed();
        }
      } else if ((green - blue > 20) && (green - red > 20) && c < 120 && yesilkapaksayisi > 0) {
        zemin = "Green";
        Stop();
        Serial.println("Green Zemin");
        forwarddelay(300);
        Stop();
        // delay(2000);
        mesafeOku();
      
        if (dis_sag < dis_sol) {  // Zeminin içinde dönmesi
          backwardsTimed(400);
          //backwardsDelayed();
        
          turnLeft();
          delay(1000);
          // turnRightDelayed();
          greenBagajOpen();
          yesilkapaksayisi = 0;
          forwarddelay(1000);
          greenBagajClosed();
          Serial.println("Sağa dönüyorum Zemin");
        } else {
          //  backwardsTimed(100);
         // backwardsDelayed();
          
          turnRight();
          delay(1000);
          greenBagajOpen();
          // turnLeftTimed(500);
          Serial.println("Sola Dönüyorum Zemin Else");
          forwarddelay(1000);
          //bagajkapatılır
          greenBagajClosed();
        }
      } else {
        state = "wallScan";
      }

      /* ----- Zemin Görmesi Finish ----- */

    }       //blue
    else {  // Eğer Başlangıç zemini Kırmızı ise
      if ((red - green > 70) && (red - blue > 75) && c > 120) {
        color = "Red";
        Stop();
        //Serial.println("kırmızı");
        frontDoorClosed();
        frontHolderRight();
        istenenkapaksayisi++;
        forwarddelay(310);
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else if ((green - blue > 20) && (green - red > 20) && c > 120) {
        color = "Green";
        //Serial.println("yesil");
        Stop();
        frontDoorClosed();
        frontHolderLeft();
        backHolderRight();
        forwarddelay(310);
        yesilkapaksayisi++;
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else if ((blue - green > 7) && (blue - red > 18) && c > 120) {
        color = "Blue";
        //Serial.println("mavi");
        Stop();
        frontDoorClosed();
        frontHolderLeft();
        backHolderLeft();
        forwarddelay(310);
        istenmeyenkapaksayisi++;
        Stop();
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      } else {
        state = "wallScan";
      }

      /* ----- Zemin Görmesi Start ----- */
      if ((red - green > 70) && (red - blue > 75) && (c < 85) && istenenkapaksayisi > 0) {
        zemin = "Red";
        Stop();
        Serial.println("Kırmızı Zemin");
        forwarddelay(310);
        Stop();
        istenenkapaksayisi = 0;
        delay(2000);
        mesafeOku();
        if (dis_sag < dis_sol) {  // Zeminin içinde dönmesi
          backwardsDelayed();
          //backwardsTimed(100);
          Stop();
          // turnLeftTimed(500);
          turnLeftDelayed();
          istenenBagajOpen();
          Serial.println("Sola Dönüyorum");
          forwarddelay(1000);
          //bagajkapatılır
          istenenBagajClosed();
        } else {
          // backwardsTimed(100);
          backwardsDelayed();
          Stop();
          // turnRightTimed(500);
          turnRightDelayed();
          istenenBagajOpen();
          forwarddelay(1000);
          //bagaj kapatılır
          istenenBagajClosed();
          Serial.println("Sağa dönüyorum");
        }
      } else if ((green - blue > 20) && (green - red > 20) && c < 120 && yesilkapaksayisi > 0) {
        zemin = "Green";
        Stop();
        Serial.println("Green Zemin");
        forwarddelay(300);
        Stop();
        yesilkapaksayisi = 0;
        delay(2000);
        mesafeOku();
        if (dis_sag < dis_sol) {  // Zeminin içinde dönmesi
          //  backwardsTimed(100);
          backwardsDelayed();
          Stop();
          turnRightDelayed();
          // turnRightTimed(500);
          greenBagajOpen();
          forwarddelay(1000);
          greenBagajClosed();
          Serial.println("Sağa dönüyorum");
        } else {
          // backwardsTimed(100);
          backwardsDelayed();
          Stop();
          turnLeftDelayed();
          greenBagajOpen();
          //turnLeftTimed(500);
          Serial.println("Sola Dönüyorum");
          forwarddelay(1000);
          //bagajkapatılır
          greenBagajClosed();
        }
      } else {
        state = "wallScan";
      }
      /* ----- Zemin Görmesi Finish ----- */

    }  //red

  }                               //collection finish
  else if (state == "release") {  // zemin bırakma
    //buraya görüntü işleme bırakma yapıalcak
  }
}





//Fonksiyonlar

void intake() {
  dxlCom.setMovingSpeed(5, 1323);
}
void intakeStop() {
  dxlCom.setMovingSpeed(5, 0);
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

  dxlCom.setTorqueEnable(5, 1);
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
  dxlCom.setTorqueEnable(16, 1);
  while (!dxlCom.dxlDataReady())
    ;
}

//DRIVETRAIN
void forward() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft);
  //Serial.print("Sol motor hızım:");
  //Serial.print(forwardLeft);
  //Serial.print("Sağ motor hızım:");
  //Serial.print(forwardRight);
}
void forwarddelay(int delayer) {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight + 400);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight + 400);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft + 400);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft + 400);
  delay(delayer);
}
void Stop() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 0);
}


void turnLeft() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 800);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 800);
  digitalWrite(LED_BUILTIN,HIGH);
}
void turnLeftTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    turnLeft();
    yeniZaman = millis();
  }

  Stop();
}

void turnRight() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 900);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 900);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 900);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 900);
  digitalWrite(LED_BUILTIN,HIGH);
}
void turnRightDelayed() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 900);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 900);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 900);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 900);
  delay(500);
}
void turnLeftDelayed() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 900);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 900);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 900);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 900);
  delay(500);
}

void turnRightTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    turnRight();
    yeniZaman = millis();
  }
  Stop();
}
void backHolderLeft() {
  dxlCom.setGoalPosition(backHolder, 690);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontDoorClosed() {
  dxlCom.setGoalPosition(frontDoor, 270);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontDoorOpen() {
  dxlCom.setGoalPosition(frontDoor, 420);
  while (!dxlCom.dxlDataReady())
    ;
}
void backHolderClosed() {
  dxlCom.setGoalPosition(backHolder, 500);
  while (!dxlCom.dxlDataReady())
    ;
}

void backHolderRight() {
  dxlCom.setGoalPosition(backHolder, 340);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontHolderLeft() {
  dxlCom.setGoalPosition(9, 645);
  while (!dxlCom.dxlDataReady())
    ;
}
void frontHolderRight() {
  dxlCom.setGoalPosition(9, 315);
  while (!dxlCom.dxlDataReady())
    ;
}
void startBagaj() {
  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], 300);
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], 700);
  //dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_90 + startFazladanAci);
}
//BAGGAGE
void istenmeyenBagajClosed() {

  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb7_180);
}

void istenenBagajClosed() {

  dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_180);
}
void istenmeyenBagajOpen() {

  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb7_90);
}

void istenenBagajOpen() {

  dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb11_90);
}

void greenBagajClosed() {

  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb6_180);
  // gb = false;
}
void greenBagajOpen() {

  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb6_90);
  // gb = false;
}
//HOLDER
void frontHolderClosed() {
  dxlCom.setGoalPosition(9, 495);
  while (!dxlCom.dxlDataReady())
    ;
}
void mesafeOku() {
  // Her sensörden mesafeyi oku
  int dis_onSol = ir_onSol.distance();
  int dis_onSag = ir_onSag.distance();
  int dis_sol = ir_sol.distance();
  int dis_sag = ir_sag.distance();

  // Sonuçları yazdır
  Serial.print("Ön Sol: ");
  Serial.println(dis_onSol);

  Serial.print("Ön Sağ: ");
  Serial.println(dis_onSag);

  Serial.print("Sol: ");
  Serial.println(dis_sol);

  Serial.print("Sağ: ");
  Serial.println(dis_sag);
}

void renkOku() {
  tcs.setInterrupt(false);  // turn on LED

  tcs.getRGB(&red, &green, &blue);
  tcs.getRawData(&r, &g, &b, &c);

  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t");
  Serial.print(int(red));
  Serial.print("\tG:\t");
  Serial.print(int(green));
  Serial.print("\tB:\t");
  Serial.print(int(blue));
  Serial.print("\tC:\t");
  Serial.print(int(c));

  Serial.print("\n");
}


void backwards() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
}
void backwardsDelayed() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
  delay(300);
}
void backwardsTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;
  while (yeniZaman - eskiZaman <= time) {
    backwards();
    yeniZaman = millis();
  }
  Stop();
}