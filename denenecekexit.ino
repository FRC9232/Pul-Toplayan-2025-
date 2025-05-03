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
#define ldr A5
#define switchpin 8

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
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int driveTrainIndex[2][2] = { { 3, 4 }, { 1, 2 } };  // sol, sag (ön, arka)
int leftMotorsIndex[2] = { driveTrainIndex[0][0], driveTrainIndex[0][1] };
int rightMotorsIndex[2] = { driveTrainIndex[1][0], driveTrainIndex[1][1] };

const int forwardLeft = 500;
const int forwardRight = forwardLeft + 1023;
const int backwardsLeft = 1524;
const int backwardsRight = backwardsLeft - 1023;
const int intakeSpeed = 700 + 1023;
const double wheelBase = 27.675 / 2;
unsigned int eskiZaman;
unsigned int yeniZaman;
unsigned int eskiZaman2 = 0;
unsigned int yeniZaman2 = 0;

unsigned long lastRenkOkuma = 0;            // Renk okuma sıklığını sınırlamak için millis.
unsigned long renkOkumaAraligi = 50; 
//-----Dynamixel------
softHalfDuplexSerial port(6);
dxliR dxlCom(&port);
//-----Dynamixel End--
int yesilkapaksayisi = 1;
int istenenkapaksayisi = 0;
int istenmeyenkapaksayisi = 0;
int maxistenmeyen = 5;
int maxgreen = 4;
int maxistenen = 5;
String startZemin = "blue";
#define backHolder 8
#define frontDoor 10
int bagajIndex[3] = { 14, 6, 16 };  //red, green, blue (11, 7, 6)
int redBagajIndex = 0;
const int greenBagajIndex = 1;
int blueBagajIndex = 2;

String color = "";
String zemin = "";

float rb7_90 = 530.0;
float bb11_90 = 400.0;  // !!! NO INFO
const float gb6_90 = 512.0;
float rb7_180 = 190.0;
float bb11_180 = 850.0;  // !!! NO INFO
const float gb6_180 = 200.0;
//const float startFazladanAci = 188;
bool state = false;
byte gelenVeri;
int switchVal;

int turnLeftDelayed_cagirma = 0;

void setup() {
pinMode(switchpin,INPUT);
  Serial.begin(115200);
  /*
  int startTime = millis();
  int nowTime = startTime;
  while (Serial.available() <= 0) {
    if(nowTime - startTime >= 2000) {break;} 
    nowTime = millis();
  }
  Serial.write(3);*/// baslangıçta yeşil armaya atama mavi için 1 kırmızı 2
  //

  //Serial.print("Basladi");
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
  torkOn();
  startBagaj();
  frontHolderClosed();
  backHolderClosed();
  frontDoorOpen();
  istenenBagajClosed();
  greenBagajClosed();
  istenmeyenBagajClosed();
  renkOku();
  if ((red - green > 70) && (red - blue > 75)) {
    startZemin = "red";
    Serial.println("Kırmızı zemin okudum kırmızı kapak topluyorum.");

  } else {
    startZemin = "blue";
    Serial.println("Mavi zemin okudum kırmızı kapak topluyorum.");
  }
  
}

void loop() {
  switchVal = digitalRead(switchpin);
  Serial.print("buton: ");
  Serial.println(switchVal);
  Serial.println(color);
  
  if (switchVal == 1) {
    StopAll();
    while (digitalRead(switchpin) == 1) {
      delay(10);
    }
  }

  robotLogic();
   

}

void StopAll() {
  Stop();
  intakeStop();
}

void robotLogic() {

  // if (state == true) {
  //   if (ir_onSol.distance() > 200 or ir_onSol.distance() > 200) {
  //     forwarddelay(500);
  //     Stop();
  //     state = false;
  //   }
  //   else{
  //      backwardsDelayed();
  //      state=false;
  //   }
  // }
  // if (millis() - lastRenkOkuma >= renkOkumaAraligi) {
  //   mesafeOku();          // Belli aralıklarla çalıştırılan renk okuma fonksiyonu.
  //   lastRenkOkuma = millis();
  //   Serial.println(color);
  //    renkOku();
  // }
  mesafeOku();
  renkOku();
 
  int isik = analogRead(ldr);
  //Serial.print("ldr değer: ");
  //Serial.println(ldr);

  int dis_onSol = ir_onSol.distance();
  int dis_onSag = ir_onSag.distance();
  int dis_sol = ir_sol.distance();
  int dis_sag = ir_sag.distance();
  
  intake();
  mesafeOku();
  if (ir_sag.distance() < 25) {
    Serial.println("Sağımda engel görüyorum");
    Stop();
    // turnRightTimed(400);
    turnLeftTimed(400);
  } else if (ir_onSol.distance() < 80) {
    Serial.println("Önümde engel görüyorum");
    Stop();

    //turnRightTimed(400);
    turnRightTimed(550);
  } else if (ir_onSag.distance() < 80) {
    Serial.println("Önümde engel görüyorum");
    Stop();
    //backwardsTimed(160);

    //turnLeftTimed(400);
    turnLeftTimed(550);

  } else if (ir_sol.distance() < 22) {
    Serial.println("solumda engel görüyorum");
    Stop();
    // turnRightTimed(360);
    turnRightTimed(400);
  } else {
    
        if (startZemin == "blue") {
          renkOku();
            
      if ((red - green > 90) && (red - blue > 95) && c > 320) {
      
        color = "Red";
        frontDoorClosed();
        Stop();
        Serial.println("kırmızı");
        frontHolderLeft();
        backHolderLeft();
        forwardTimed(330);
        istenmeyenkapaksayisi++;
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      }
      else if ((blue-green>5)&&(blue-red>-7)&&(blue-red<25) && c > 220) {
  
        color = "Blue";
        Serial.println("mavi pul");
        frontDoorClosed();
        Stop();
        if (istenenkapaksayisi == maxistenen) {
          
          frontHolderLeft();
          backHolderLeft();
          forwardTimed(310);
          istenmeyenkapaksayisi++;
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        } else if (istenenkapaksayisi < maxistenen) {
           Stop();
          frontHolderRight();
          istenenkapaksayisi++;
          forwardTimed(310);
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        }
      } else if ((green - blue > 25) && (green - red > 10) && c > 180) {
        color = "Green";
        Serial.println("yesil pul");
        frontDoorClosed();
        Stop();
        if (yesilkapaksayisi >= maxgreen) {
           Stop();
          frontHolderLeft();
          backHolderLeft();
          forwardTimed(310);
          istenmeyenkapaksayisi++;
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        } else if (yesilkapaksayisi < maxgreen) {
           Stop();
          frontHolderLeft();
          backHolderRight();
          forwardTimed(330);
          yesilkapaksayisi++;
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        }
      }
else if ((green - blue > 25) && (green - red > 10) && c < 180 && yesilkapaksayisi>0) {
        zemin = "Green";
        Stop();
        Serial.println("Green Zemin");

        forwardTimed(100);
        Stop();
        // delay(2000);
        mesafeOku();
        if (dis_sag < 220) {  // Sağ taraf yaklaştıysa, zeminde
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();  // Sensörleri oku
              turnLeft();   // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                greenBagajOpen();
                yesilkapaksayisi = 0;
                forwardTimed(550);
                greenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        } else {// Sol taraf duvara daha yakınsa
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();  // Sensörleri oku
              turnRight();  // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                greenBagajOpen();
                yesilkapaksayisi = 0;
                forwardTimed(350);
                greenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        }
      }
else if ((blue-green>5)&&(blue-red>-7)&&(blue-red<25)  && c < 220 && istenenkapaksayisi > 0) {
        zemin = "Blue";
        Stop();
        forwardTimed(100);
        Stop();
        Serial.println("Blue Zemin");
        // forwarddelay(200);

        // delay(2000);
        mesafeOku();
        if (dis_sag < 220) {  // Sağ taraf yaklaştıysa, zeminde
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();
              // Sensörleri oku
              turnLeft();  // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                istenenBagajOpen();
                istenenkapaksayisi = 0;
                forwardTimed(550);
                istenenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        } else {//Sol taraf duvara daha yakınsa
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();  // Sensörleri oku
              turnRight();  // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                istenenBagajOpen();
                istenenkapaksayisi = 0;
                forwardTimed(750);
                istenenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        }
      }
      else {
          if (istenmeyenkapaksayisi >= maxistenmeyen) {
        istenmeyenBagajOpen();
        istenmeyenkapaksayisi = 0;
        frontDoorClosed();
        forwardTimed(330);
        istenmeyenBagajClosed();
        frontDoorOpen();
      }
       frontDoorOpen();
        frontHolderClosed();
        backHolderClosed();
        forward();
        mesafeOku();
        renkOku();
        color = "White";
      }
        
  }  
  else{// Kırmızı zeminse
  renkOku();
     if ((red - green > 90) && (red - blue > 95) && c > 320) {
        color = "Red";
        frontDoorClosed();
        Stop();
        if (istenenkapaksayisi >= maxistenen) {
           Stop();

          frontHolderLeft();
          backHolderLeft();
          forwardTimed(310);
          istenmeyenkapaksayisi++;
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        } else if (istenenkapaksayisi < maxistenen) {
            Serial.println("Kırmızı kapak istenene gidiyor.");
             Stop();
          frontHolderRight();
          istenenkapaksayisi++;
          forwardTimed(310);
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        }
        //Serial.println("kırmızı");

      }
      else if ((green - blue > 25) && (green - red > 10) && c > 180) {
        color = "Green";
        Serial.println("yesil pul");
        frontDoorClosed();
        Stop();
        if (yesilkapaksayisi == maxgreen) {
           Stop();
          frontHolderLeft();
          backHolderLeft();
          forwardTimed(310);
          istenmeyenkapaksayisi++;
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        } else if (yesilkapaksayisi < maxgreen) {
          Stop();
          frontHolderLeft();
          backHolderRight();
          forwardTimed(340);
          yesilkapaksayisi++;
          frontHolderClosed();
          backHolderClosed();
          frontDoorOpen();
        }
      }
      else if ((blue-green>5)&&(blue-red>-7)&&(blue-red<25) && c > 220) {
        color = "Blue";
        frontDoorClosed();
        Stop();
        Serial.println("blue pul");
        frontHolderLeft();
        backHolderLeft();
        forwardTimed(310);
        istenmeyenkapaksayisi++;
        frontHolderClosed();
        backHolderClosed();
        frontDoorOpen();
      }

else if ((green - blue > 25) && (green - red > 10) && c < 120 && yesilkapaksayisi > 0) {
        zemin = "Green";
        Stop();
        Serial.println("Green Zemin");
forwardTimed(100);
        Stop();

        // delay(2000);
        mesafeOku();
        if (dis_sag < 220) {  // Sağ taraf yaklaştıysa, zeminde
          unsigned long previousMillis = millis();
          
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();  // Sensörleri oku
              turnLeft();   // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                greenBagajOpen();
                yesilkapaksayisi = 0;
                forwardTimed(550);
                greenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        } else {// Sol taraf duvara daha yakınsa
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();  // Sensörleri oku
              turnRight();  // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  ))  {
                Stop();
                greenBagajOpen();
                yesilkapaksayisi = 0;
                forwardTimed(750);
                greenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        }
      }
    else if ((red - green > 70) && (red - blue > 75) && c < 120 && istenenkapaksayisi>0) {
        zemin = "Red";
        Stop();
        forwardTimed(100);
        Stop();

        Serial.println("Red Zemin");
        // forwarddelay(200);

        // delay(2000);
        mesafeOku();
        if (dis_sag < 220) {  // Sağ taraf yaklaştıysa, zeminde
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();
              // Sensörleri oku
              turnLeft();  // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                istenenBagajOpen();
                istenenkapaksayisi = 0;
                forwardTimed(750);
                istenenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        } else {//Sol taraf duvara daha yakınsa
          unsigned long previousMillis = millis();
          const unsigned long interval = 50;  // Sensörleri her 50 ms'de bir kontrol et

          while (true) {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval) {
              previousMillis = currentMillis;

              mesafeOku();  // Sensörleri oku
              turnRight();  // Sürekli dön

              if (ir_onSol.distance() > 340 && ir_onSag.distance() > 340 &&(ir_sag.distance() < 65 or ir_sol.distance() < 65  )) {
                Stop();
                istenenBagajOpen();
                istenenkapaksayisi = 0;
                forwardTimed(750);
                istenenBagajClosed();
                break;  // İş bitince while'dan çık
              }
            }
          }
        }
      }
      else {
         if (istenmeyenkapaksayisi >= maxistenmeyen) {
        istenmeyenBagajOpen();
        istenmeyenkapaksayisi = 0;
        frontDoorClosed();
        forwardTimed(330);
        istenmeyenBagajClosed();
        frontDoorOpen();
      }
       frontDoorOpen();
        frontHolderClosed();
        backHolderClosed();
        forward();
        mesafeOku();
        renkOku();
        color = "White";
      }

      }
    
  
}

}

void intake() {
  dxlCom.setMovingSpeed(5, 1423);
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
void forwarddash(){
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight + 400);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight + 400);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft + 400);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft + 400);

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

  //5 saniyeye kadar yapıcakları
  frontDoorClosed();
  frontHolderLeft();
  backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 990);

  
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
void forwardTimed(int time) {
  eskiZaman = millis();
  yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= time) {
    forwarddash();
    yeniZaman = millis();
  }

  Stop();
}

void turnRight() {
  frontDoorClosed();
  frontHolderLeft();
  backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 999);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 999);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 999);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 999);
}
void turnRightDelayed() {
  frontDoorClosed();
  frontHolderLeft();
  backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 990);

  delay(500);
}
void turnLeftDelayed() {

  frontDoorClosed();
  frontHolderLeft();
  backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 990);
  delay(350);
}
void turnLeftDelayed2() {
  frontDoorClosed();
  frontHolderLeft();
  backHolderLeft();
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 990);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 990);
  delay(700);
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

void timeOut(int time) {
  yeniZaman2 = millis();
  //5 saniyeye kadar yapıcakları
  while (yeniZaman2 - eskiZaman2 >= 5000) {
    //5 saniye sonra yapılacak olan işlemler
    state = "exit";
    eskiZaman2 = yeniZaman2;
  }
}