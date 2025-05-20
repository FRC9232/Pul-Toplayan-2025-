#include <Wire.h>
#include <DynamixeliR.h>
#include <DynamixelAx.h>
#include <SoftHalfDuplexSerial.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include "Wire.h"


///////////////////////////////////////////////////////////////////////////////
                                    //TCS//

#define redpin 3             // TCS34725 Red pin tanımlanması.
#define greenpin 5           // TCS34725 Green pin tanımlanması.
#define bluepin 6            // TCS34725 Blue pin tanımlanması.
#define commonAnode true

float red, green, blue; // RGB değerleri.
int r,g,b,c;            // RAW değerleri.

String renk = "";       // Renk statment (opsiyonel).
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // TCS objesinin tanımının yapılması.

unsigned long lastRenkOkuma = 0;            // Renk okuma sıklığını sınırlamak için millis.
unsigned long renkOkumaAraligi = 25;  // Renk okuma sıklığı için aralık.
bool greenBagajBosalt = false;
bool blueBagajBosalt = false;
bool greenBagajBosalt2 = false;
bool redBagajBosalt2 = false;
bool onGround = false;
bool dashBool;
bool kabinBool = false;
unsigned int eskiZaman;
unsigned int yeniZaman;
int lastKabin = 0;


///////////////////////////////////////////////////////////////////////////////
                                //Gyro ve PID

///////////////////////////////////////////////////////////////////////////////
                            //Drivetrain Ayarları//


const int forwardSpeed = 500;
const int backwardSpeed = 500;
const int turningSpeed = 500;

const int turnRspeedOfR = 250;    
const int turnRspeedOfL = 250;               
const int turnLspeedOfR = 1273;
const int turnLspeedOfL = 1273;

const int forwardLeft = 450;
const int forwardRight = forwardLeft + 1023;
const int backwardsLeft = 1524;
const int backwardsRight = backwardsLeft - 1023;
const int intakeSpeed = 1000;

int driveTrainIndex[2][2] = { { 1, 4 }, { 3, 2 } };                           // sol, sag (ön, arka)
int leftMotorsIndex[2] = { driveTrainIndex[0][0], driveTrainIndex[0][1] };    // Smart drive için motorlar sağ ve sol olarak ayrılıyor.
int rightMotorsIndex[2] = { driveTrainIndex[1][0], driveTrainIndex[1][1] };


///////////////////////////////////////////////////////////////////////////////
                          //Dynamixel Ayarları//


softHalfDuplexSerial port(10);  // Dynamixelin arduinodaki 10. porta bağlanması.
dxliR dxlCom(&port);            // dxlCom objesinin oluşturulması.


///////////////////////////////////////////////////////////////////////////////
                          //Kapı ve Yönlendiriciler//


#define leftHolder 6    // 6 numaralı yönlendirici motorunun id tanımı.
#define rightHolder 7   // 7 numaralı yönlendirici motorunun id tanımı.
#define frontDoor 12    // 11 numaralı kapı motorunun id tanımı.
#define backDoor 11

const int rhGreen = 494;   // Yeşil kabinin sağ yönlendiricisinin yönlendirme değeri.
const int lhGreen = 572;   // Yeşil kabinin sol yönlendiricisinin yönlendirme değeri.
const int rhBlue = 456;    // Mavi kabinin sağ yönlendiricisinin yönlendirme değeri.
const int lhBlue = 800;    // Mavi kabinin sol yönlendiricisinin yönlendirme değeri.
const int rhRed = 304;     // Kırmızı kabinin sağ yönlendiricisinin yönlendirme değeri.
const int lhRed = 624;     // Kırmızı kabinin sol yönlendiricisinin yönlendirme değeri.


const int rhRed2 = 456;    // Mavi kabinin sağ yönlendiricisinin yönlendirme değeri.
const int lhRed2 = 800;    // Mavi kabinin sol yönlendiricisinin yönlendirme değeri.
const int rhBlue2 = 304;     // Kırmızı kabinin sağ yönlendiricisinin yönlendirme değeri.
const int lhBlue2 = 624;     // Kırmızı kabinin sol yönlendiricisinin yönlendirme değeri.





///////////////////////////////////////////////////////////////////////////////
                            //Pul Ayarları//


const int maximumBluePul = 5;        // Maksimum mavi pul sayısı.
const int maximumRedPul = 3;         // Maksimum kırmızı pul sayısı.
const int maximumGreenPul = 3;       // Maksimum yeşil pul sayısı.

const int maximumBluePul2 = 3;        // Maksimum mavi pul sayısı.
const int maximumRedPul2 = 5;         // Maksimum kırmızı pul sayısı.
const int maximumGreenPul2 = 3;

int bluePulCounter = 0;              // Başlangıç mavi pul sayısı.
int redPulCounter = 0;               // Başlangıç kırmızı pul sayısı.
int bluePulCounter2 = 0;              // Başlangıç mavi pul sayısı.
int redPulCounter2 = 0;               // Başlangıç kırmızı pul sayısı.
int greenPulCounter = 0;             // Başlangıç yeşil pul sayısı.
int greenPulCounter2 = 0;             // Başlangıç yeşil pul sayısı.


unsigned long pulLoadingTime = 1100; // Pulun kabinlere yüklenme süresi.
unsigned long pulNormalTime = 50;    // Renk sensörü default renk okuma eşiği.
int deger;
String startzemin = "";


///////////////////////////////////////////////////////////////////////////////
                            //Bagaj Tanımlama//


int bagajIndex[3] = { 8, 9, 10 };  // Bagajların index tanımı (idris ne uğraştın böyle şeylerle gülüm ya).
int redBagajIndex = 2;             // Kırmızı bagaj indexi.
const int greenBagajIndex = 1;     // Yeşil bagaj indexi.
int blueBagajIndex = 0;            // Mavi bagaj indexi.

int bagajIndex2[3] = { 8, 9, 10 };  // Bagajların index tanımı (idris ne uğraştın böyle şeylerle gülüm ya).
int redBagajIndex2 = 0;             // Kırmızı bagaj indexi.
const int greenBagajIndex2 = 1;     // Yeşil bagaj indexi.
int blueBagajIndex2 = 2;            // Mavi bagaj indexi.


///////////////////////////////////////////////////////////////////////////////
                        //Bagaj Değişken Ayarları//


bool isRedBagajOpen = false;
bool isBlueBagajOpen = false;
bool isRedBagajOpen2 = false;
bool isBlueBagajOpen2 = false;
bool isGreenBagajOpen = false;
bool isGreenBagajOpen2 = false;

float rb10_90 = 436;        // Kırmızı bagajın kapatılma offseti.
float bb8_90 = 917;         // Mavi bagajın kapatılma offseti.
const float gb9_90 = 685;   // Yeşil bagajın kapatılma offseti.

float rb10_180 = 900;       // İstenmeyen bagajın açılma offseti.
float bb8_180 = 458;        // İstenen bagajın açılma offseti.
float gb9_180 = 225;        // Yeşil bagajın açılma offsetii




float bb10_90_2 = 436;        // Kırmızı bagajın kapatılma offseti.
float rb8_90_2 = 917;         // Mavi bagajın kapatılma offseti.

float bb10_180_2 = 900;       // İstenmeyen bagajın açılma offseti.
float rb8_180_2 = 458;        // İstenen bagajın açılma offseti.

///////////////////////////////////////////////////////////////////////////////
                            //MZ 80 ayarları//
const int solOnSensorPin = 2;   // MZ 80 sol ön sensör id tanımı.
const int solYanSensorPin = 3;   // MZ 80 sol yan sensör id tanımı.
const int sagOnSensorPin = 4;   // MZ 80 sağ ön sensör id tanımı.
const int sagYanSensorPin = 5;   // MZ 80 sağ yan sensör id tanımı.
///////////////////////////////////////////////////////////////////////////////
                                //Setup//
void setup() {

  Serial.begin(9600);       // Seri iletişim baund rate.
  dxlCom.begin(57600);      // Dynamixel iletişim baund rate.
  Serial.print("Basladi");
  pinMode(solOnSensorPin, INPUT);   // MZ 80 sensörünün input olarak tanımlanması.
  pinMode(sagOnSensorPin, INPUT);   // MZ 80 sensörünün input olarak tanımlanması.
  pinMode(solYanSensorPin, INPUT);   // MZ 80 sensörünün input olarak tanımlanması.
  pinMode(sagYanSensorPin, INPUT);   // MZ 80 sensörünün input olarak tanımlanması.
  
  //torkOn();           // Bütün motorlara tork verilmesi.
  tcsSetup();         // Renk sensörünün hazırlanması.
  delay(2000);
  startZemin();
  

}
///////////////////////////////////////////////////////////////////////////////
                                 //Loop//
void loop() {
  
  if (millis() - lastRenkOkuma >= renkOkumaAraligi) {
    renkOku();          // Belli aralıklarla çalıştırılan renk okuma fonksiyonu.
    lastRenkOkuma = millis();
  }

  intake();

  if(startzemin == "blue"){
    if(renk == "red"){
      if(isRedBagajOpen){
        redBagajClosed();
      }
      renk = "";
      renkOkumaAraligi = pulLoadingTime;
      frontDoorOpen();
      Stop();
      Serial.println("Kırmızı renk karar mekanizması çalışıyor.");

      if(redPulCounter < maximumRedPul) {
        redPulCounter ++;
        Serial.println("Kırmızı renk pul sayısı maksimuma ULAŞMAMIŞ !");
        redKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();
        defaultKabin();
      }
      else {
        Serial.println("Kırmızı renk pul sayısı maksimuma ULAŞMIŞ !");
        redKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();                    // Yeni pul geçişinin sağlanması için kapının kapalı konuma getirilmesi.
        redBagajOpen();                       // Mavş pul sayısının maksimuma ulaşmasından dolayı fazladan toplanan pul / pulların boşaltılması.
        defaultKabin();
      }
    }

    if(renk == "blue"){
      if(isRedBagajOpen || isBlueBagajOpen){
        redBagajClosed();
        blueBagajClosed();
      }
      renk = "";
      renkOkumaAraligi = pulLoadingTime;
      frontDoorOpen();
      Stop();
      Serial.println("Mavi renk karar mekanizması çalışıyor.");
      if(bluePulCounter < maximumBluePul) {
        bluePulCounter ++;
        Serial.println("Mavi renk pul sayısı maksimuma ULAŞMAMIŞ !");
        blueKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();             // Mavi pul yönlendirici motorlarının deaktif olduğunu belirten mantıksal ifade.
        defaultKabin();
      }
      else {
        Serial.println("Mavi renk pul sayısı maksimuma ULAŞMIŞ !");
        redKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();                    // Yeni pul geçişinin sağlanması için kapının kapalı konuma getirilmesi.
        redBagajOpen();                       // Mavş pul sayısının maksimuma ulaşmasından dolayı fazladan toplanan pul / pulların boşaltılması.
        defaultKabin();
      }
    }

    if(renk == "green"){
      if(isRedBagajOpen || isGreenBagajOpen){
        redBagajClosed();
        greenBagajClosed();
      }
      renk = "";
      renkOkumaAraligi = pulLoadingTime;
      frontDoorOpen();
      Stop();
      Serial.println("Yeşil renk karar mekanizması çalışıyor.");

      if(greenPulCounter < maximumGreenPul) {
        greenPulCounter ++;
        Serial.println("Yeşil renk pul sayısı maksimuma ULAŞMAMIŞ !");
        greenKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();
        defaultKabin();
      }
      else {
        Serial.println("Yeşil renk pul sayısı maksimuma ULAŞMIŞ !");
        redKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();                    // Yeni pul geçişinin sağlanması için kapının kapalı konuma getirilmesi.
        redBagajOpen();                       // Mavş pul sayısının maksimuma ulaşmasından dolayı fazladan toplanan pul / pulların boşaltılması.
        defaultKabin();
      }
    }
    forward2();
  }

  else if(startzemin == "red"){
    if(renk == "red"){
      if(isRedBagajOpen2){
        redBagajClosed2();
      }
      renk = "";
      renkOkumaAraligi = pulLoadingTime;
      frontDoorOpen();
      Stop();
      Serial.println("Kırmızı renk karar mekanizması çalışıyor.");

      if(redPulCounter2 < maximumRedPul2) {
        redPulCounter2 ++;
        Serial.println("Kırmızı renk pul sayısı maksimuma ULAŞMAMIŞ !");
        redKabin2();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();
        defaultKabin();
      }
      else {
        Serial.println("Kırmızı renk pul sayısı maksimuma ULAŞMIŞ !");
        blueKabin2();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();                    // Yeni pul geçişinin sağlanması için kapının kapalı konuma getirilmesi.
        blueBagajOpen2();                       // Mavş pul sayısının maksimuma ulaşmasından dolayı fazladan toplanan pul / pulların boşaltılması.
        defaultKabin();
      }
    }

    if(renk == "blue"){
      if(isRedBagajOpen2 || isBlueBagajOpen2){
        redBagajClosed2();
        blueBagajClosed2();
      }
      renk = "";
      renkOkumaAraligi = pulLoadingTime;
      frontDoorOpen();
      Stop();
      Serial.println("Mavi renk karar mekanizması çalışıyor.");
      if(bluePulCounter2 < maximumBluePul2) {
        bluePulCounter2 ++;
        Serial.println("Mavi renk pul sayısı maksimuma ULAŞMAMIŞ !");
        blueKabin2();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();             // Mavi pul yönlendirici motorlarının deaktif olduğunu belirten mantıksal ifade.
        defaultKabin();
      }
      else {
        Serial.println("Mavi renk pul sayısı maksimuma ULAŞMIŞ !");
        blueKabin2();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();                    // Yeni pul geçişinin sağlanması için kapının kapalı konuma getirilmesi.
        blueBagajOpen2();                       // Mavş pul sayısının maksimuma ulaşmasından dolayı fazladan toplanan pul / pulların boşaltılması.
        defaultKabin();
      }
    }

    if(renk == "green"){
      if(isRedBagajOpen2 || isGreenBagajOpen2){
        redBagajClosed2();
        greenBagajClosed2();
      }
      renk = "";
      renkOkumaAraligi = pulLoadingTime;
      frontDoorOpen();
      Stop();
      Serial.println("Yeşil renk karar mekanizması çalışıyor.");

      if(greenPulCounter2 < maximumGreenPul2) {
        greenPulCounter2 ++;
        Serial.println("Yeşil renk pul sayısı maksimuma ULAŞMAMIŞ !");
        greenKabin();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();
        defaultKabin();
      }
      else {
        Serial.println("Yeşil renk pul sayısı maksimuma ULAŞMIŞ !");
        blueKabin2();
        backDoorOpen();
        forwarddelay(550);
        Stop();
        backDoorClosed();
        frontDoorClosed();                    // Yeni pul geçişinin sağlanması için kapının kapalı konuma getirilmesi.
        blueBagajOpen2();                       // Mavş pul sayısının maksimuma ulaşmasından dolayı fazladan toplanan pul / pulların boşaltılması.
        defaultKabin();
      }
    }
    forward3();
  }
}


///////////////////////////////////////////////////////////////////////////////
                          //Intake fonksiyonları//


void startZemin(){
  tcs.setInterrupt(false);             // Ledin açılması.
  tcs.getRGB(&red, &green, &blue);     // RGB değerlerinin okunması.
  tcs.getRawData(&r, &g, &b, &c);      // RAW değerlerinin okunması.
  tcs.setInterrupt(true);              // Ledin kapatılması.

  if((red-green>70)&&(red-blue>75)&&(c<250)){
    startzemin = "red";
    robotDefault2();
    Serial.println("Kırmızı start zemin");
  }
  else if((blue-green>7)&&(blue-red>7)&&(c<250)){
    startzemin = "blue";
    robotDefault();
    Serial.println("Mavi start zemin tespit edildi.");
  }


}
void intake() {
  dxlCom.setMovingSpeed(5, 950);
} // Intake motorunu çalıştırma fonksiyonu.
void intakeStop() {
  dxlCom.setMovingSpeed(5, 0);
} // Intake motorunu durdurma fonksiyonu.
void outtake(){
  dxlCom.setMovingSpeed(5, 950+1023);
} // Intake motorunu ters yönde çalıştırma fonksiyonu.


///////////////////////////////////////////////////////////////////////////////
                          //Detection fonksiyonları//







///////////////////////////////////////////////////////////////////////////////
                        //Renk Okuma Fonksiyonu (TCS)//


void renkOku() {
  tcs.setInterrupt(false);             // Ledin açılması.
  tcs.getRGB(&red, &green, &blue);     // RGB değerlerinin okunması.
  tcs.getRawData(&r, &g, &b, &c);      // RAW değerlerinin okunması.
  tcs.setInterrupt(true);              // Ledin kapatılması.

  
  if((red>180)&&(red-green>140)&&(red-blue>140)&&(c>870)){
    renk = "red";
    Serial.println("Kırmızı kapak tespit edildi.");
  }
  else if((blue-green>5)&&(blue-red>-7)&&(blue-red<15)&&(c>400)){
    renk = "blue";
    Serial.println("Mavi kapak tespit edildi.");
  }

  else if((green>85)&&(green-blue>30)&&(c>300)&&(c<520)){
    renk = "green";
    Serial.println("Yeşil kapak tespit edildi.");
    Serial.println(renk);
  }

  else if((red-green>70)&&(red-blue>75)&&(c<250)){
    renk = "redground";
    onGround = true;
    //Serial.println("Kırmızı zemin tespit edildi.");
  }
  else if((blue-green>7)&&(blue-red>7)&&(c<250)){
    renk = "blueground";
    onGround = true;
    Serial.println("Mavi zemin tespit edildi.");
  }

  else if((green-blue>20)&&(c<200)){
    renk = "greenground";
    onGround = true;
    Serial.println("Yeşil zemin tespit edildi.");
  }

  else if((red-blue>30)){
    renk = "whiteground";
    onGround = false;
    dashBool = true;
    //Serial.println("Beyaz zemin tespit edildi.");
  }
} // RGB değerlerine göre renkleri okuyup karar mekanizmalarını döndüren fonksiyon


///////////////////////////////////////////////////////////////////////////////
                            //Tork Fonksiyonu//

void torkOn() {
  Serial.println("Motorlara tork veriliyor.");
  dxlCom.setTorqueEnable(1, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(2, 1);

  while (!dxlCom.dxlDataReady())
    ;

  dxlCom.setTorqueEnable(3, 1);

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

   dxlCom.setTorqueEnable(7, 1);

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

  dxlCom.setTorqueEnable(12, 1);

  while (!dxlCom.dxlDataReady())
    ;


  dxlCom.setTorqueEnable(4, 1);

  while (!dxlCom.dxlDataReady())
    ;  

  Serial.println("Motorlara başarıyla tork verildi.");
} // Motorlara tork veren fonksiyon.


///////////////////////////////////////////////////////////////////////////////
                        //Drivetrain (pidsiz) Fonksiyonlar //


void forward() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft);
}  // İleri yönde hareket sağlayan fonksiyon



void forward2() {
  
  int solOnMesafe = digitalRead(solOnSensorPin);
  int solYanMesafe = digitalRead(solYanSensorPin); // "sol arka" tanımınız "sol yan" olarak düzeltildi.
  int sagOnMesafe = digitalRead(sagOnSensorPin);
  int sagYanMesafe = digitalRead(sagYanSensorPin); // "sağ arka" tanımınız "sağ yan" olarak düzeltildi.
  if(onGround&&dashBool){
    forwarddelay(300);
    dashBool = false;
  }

  if((solOnMesafe == LOW)||(sagOnMesafe == LOW)||(solYanMesafe == LOW)||(sagYanMesafe == LOW)){
    //Serial.println("Engel.");
    if((!onGround)&&(solOnMesafe == LOW)&&(sagOnMesafe == LOW)){
      Stop();
      backwards();
      delay(350);
      turnRight();
      delay(500);
      Stop();
    }
    if((renk=="greenground")||(renk=="blueground")){
      Stop();
      Serial.println("dönüyom");
      turnLeftPower();
      if((renk=="greenground")&&(greenPulCounter > 0)){
        greenBagajBosalt = true;
      }
      else if((renk=="blueground")&&(bluePulCounter > 0)){
        blueBagajBosalt = true;
      }
    }
    if (((sagOnMesafe == LOW)||(sagYanMesafe == LOW))&&(solYanMesafe != LOW)&&(solOnMesafe != LOW)){
    //backwardsdelay();
      if((sagOnMesafe == LOW)&&(solOnMesafe != LOW)&&(sagYanMesafe != LOW)&&(solYanMesafe != LOW)){
        turnLeftDelay();
      }
      else{
        turnLeft();
      }
    }
    else if(((solOnMesafe == LOW)||(solYanMesafe == LOW))&&(sagYanMesafe != LOW)&&(sagOnMesafe != LOW)){
      if((solOnMesafe == LOW)&&(sagOnMesafe != LOW)&&(sagYanMesafe != LOW)&&(solYanMesafe != LOW)){
        turnRightDelay();
      }
      else{
        turnRight();
      }
    }
  }
  else{
    if((blueBagajBosalt == true || greenBagajBosalt == true)&&(solOnMesafe != LOW)&&(sagOnMesafe != LOW)&&((solYanMesafe != LOW)||(sagYanMesafe != LOW))){
      Stop();
      if(blueBagajBosalt == true){
        blueBagajOpen();
        blueBagajBosalt = false;
      }
      else{
        greenBagajOpen();
        greenBagajBosalt = false;
      } 
    }
    else{
      forward();
    }   
  }
}


void forward4() {
  
  int solOnMesafe = digitalRead(solOnSensorPin);
  int solYanMesafe = digitalRead(solYanSensorPin); // "sol arka" tanımınız "sol yan" olarak düzeltildi.
  int sagOnMesafe = digitalRead(sagOnSensorPin);
  int sagYanMesafe = digitalRead(sagYanSensorPin); // "sağ arka" tanımınız "sağ yan" olarak düzeltildi.

  if((solOnMesafe == LOW)||(sagOnMesafe == LOW)||(solYanMesafe == LOW)||(sagYanMesafe == LOW)){
    //Serial.println("Engel.");
    if((renk=="greenground")||(renk=="blueground")){
      Stop();
      Serial.println("dönüyom");
      turnLeftPower();
      if((renk=="greenground")&&(greenPulCounter > 0)){
        greenBagajBosalt = true;
      }
      else if((renk=="blueground")&&(bluePulCounter > 0)){
        blueBagajBosalt = true;
      }
    }
    if (((sagOnMesafe == LOW)||(sagYanMesafe == LOW))&&(solYanMesafe != LOW)&&(solOnMesafe != LOW)){
    //backwardsdelay();
      if((sagOnMesafe == LOW)&&(solOnMesafe != LOW)&&(sagYanMesafe != LOW)&&(solYanMesafe != LOW)){
        turnLeftDelay();
      }
      else{
        turnLeft();
      }
    }
    else if(((solOnMesafe == LOW)||(solYanMesafe == LOW))&&(sagYanMesafe != LOW)&&(sagOnMesafe != LOW)){
      if((solOnMesafe == LOW)&&(sagOnMesafe != LOW)&&(sagYanMesafe != LOW)&&(solYanMesafe != LOW)){
        turnRightDelay();
      }
      else{
        turnRight();
      }
    }
  }
  else{
    if((blueBagajBosalt == true ||greenBagajBosalt == true)&&(solOnMesafe != LOW)&&(sagOnMesafe != LOW)&&((solYanMesafe != LOW)||(sagYanMesafe != LOW))){
      Stop();
      
      
     
      if(blueBagajBosalt == true){
        blueBagajOpen();
        blueBagajBosalt = false;
      }
      else{
        greenBagajOpen();
        greenBagajBosalt = false;
      }
    }
    
    else{
      forward();
    }
  }
}

void forward3() {
  
  int solOnMesafe = digitalRead(solOnSensorPin);
  int solYanMesafe = digitalRead(solYanSensorPin); // "sol arka" tanımınız "sol yan" olarak düzeltildi.
  int sagOnMesafe = digitalRead(sagOnSensorPin);
  int sagYanMesafe = digitalRead(sagYanSensorPin); // "sağ arka" tanımınız "sağ yan" olarak düzeltildi.
  if(onGround&&dashBool){
    forwarddelay(300);
    dashBool = false;
  }

  if((solOnMesafe == LOW)||(sagOnMesafe == LOW)||(solYanMesafe == LOW)||(sagYanMesafe == LOW)){
    //Serial.println("Engel.");
    if((!onGround)&&(solOnMesafe == LOW)&&(sagOnMesafe == LOW)){
      Stop();
      backwards();
      delay(350);
      turnRight();
      delay(500);
      Stop();
    }
    if((renk=="greenground")||(renk=="redground")){
      Stop();
      Serial.println("dönüyom");
      turnLeftPower();
      if((renk=="greenground")&&(greenPulCounter2 > 0)){
        greenBagajBosalt2 = true;
      }
      else if((renk=="redground")&&(redPulCounter2 > 0)){
        redBagajBosalt2 = true;
      }
    }
    if (((sagOnMesafe == LOW)||(sagYanMesafe == LOW))&&(solYanMesafe != LOW)&&(solOnMesafe != LOW)){
    //backwardsdelay();
      if((sagOnMesafe == LOW)&&(solOnMesafe != LOW)&&(sagYanMesafe != LOW)&&(solYanMesafe != LOW)){
        turnLeftDelay();
      }
      else{
        turnLeft();
      }
    }
    else if(((solOnMesafe == LOW)||(solYanMesafe == LOW))&&(sagYanMesafe != LOW)&&(sagOnMesafe != LOW)){
      if((solOnMesafe == LOW)&&(sagOnMesafe != LOW)&&(sagYanMesafe != LOW)&&(solYanMesafe != LOW)){
        turnRightDelay();
      }
      else{
        turnRight();
      }
    }
  }
  else{
    if((redBagajBosalt2 == true || greenBagajBosalt2 == true)&&(solOnMesafe != LOW)&&(sagOnMesafe != LOW)&&((solYanMesafe != LOW)||(sagYanMesafe != LOW))){
      Stop();
      if(redBagajBosalt2 == true){
        redBagajOpen2();
        redBagajBosalt2 = false;
      }
      else{
        greenBagajOpen2();
        greenBagajBosalt2 = false;
      } 
    }
    else{
      forward();
    }   
  }
}

void backwards() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
} // Geri yönde hareket sağlayan fonksiyon.

void backwarddelay() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], backwardsRight);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], backwardsRight);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], backwardsLeft);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], backwardsLeft);
  delay(200); // Geri gitme süresi ayarlanabilir
}

void forwarddelay(int x) {
  unsigned long eskiZaman = millis();
  unsigned long yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= x) {
    yeniZaman = millis(); // Her döngüde yeniZaman'ı güncelle

    int solOnMesafe = digitalRead(solOnSensorPin);
    int sagOnMesafe = digitalRead(sagOnSensorPin);

    if ((solOnMesafe == LOW) || (sagOnMesafe == LOW)) {
      Stop();
      Serial.println("Ön sensör engel algıladı. Sağa dönülüyor.");
      turnRight(); // Sağa dönme fonksiyonunu çağır
      delay(500);  // 500 milisaniye bekle
      Stop();
      return; // Fonksiyondan çık, ileri hareket durduruldu
    } else {
      forwarddash();
    }
  }
}

void forwarddelay1() {
  unsigned long eskiZaman = millis();
  unsigned long yeniZaman = eskiZaman;

  while (yeniZaman - eskiZaman <= 550) {
    yeniZaman = millis(); // Her döngüde yeniZaman'ı güncelle

    int solOnMesafe = digitalRead(solOnSensorPin);
    int sagOnMesafe = digitalRead(sagOnSensorPin);

    if ((solOnMesafe == LOW) || (sagOnMesafe == LOW)) {
      Serial.println("Ön sensör engel algıladı. Duruluyor.");
      Stop();
      backwards();
      delay(250);
      Stop();
      return; // Fonksiyondan çık, ileri hareket durduruldu
    } else {
      forwarddash();
    }
  }
}


void forwarddash() {
  Serial.println("Robot dash atıyor.");
  dxlCom.setMovingSpeed(rightMotorsIndex[0], forwardRight + 320);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], forwardRight + 320);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], forwardLeft + 320);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], forwardLeft + 320);
  
} // 300 ms boyunca (delay ile) ileri yönde hareket sağlayan fonksiyon.

void Stop() {
  Serial.println("Robot durdu");
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 0);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 0);
} // Hareketi durduran fonksiyon.

void turnLeft() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 800);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 800);
  

} // Sol yönünde hareket sağlayan fonksiyon.


void turnLeftPower() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 1024 + 1000);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 1024 + 1000);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 1024 + 1000);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 1024 + 1000);
  

}

void turnRight() {
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 800);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 800);
} // Sağ yönde hareket sağlayan fonksiyon.

void turnRightDelay() {
  deger = random(300,680);
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 800);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 800);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 800);
  delay(deger);
  Stop();
} // Sağ yönde hareket sağlayan fonksiyon.

void turnLeftDelay() {
  deger = random(300,680);
  dxlCom.setMovingSpeed(rightMotorsIndex[0], 800+1024);
  dxlCom.setMovingSpeed(rightMotorsIndex[1], 800+1024);
  dxlCom.setMovingSpeed(leftMotorsIndex[0], 800+1024);
  dxlCom.setMovingSpeed(leftMotorsIndex[1], 800+1024);
  delay(deger);
  Stop();
} // Sağ yönde hareket sağlayan fonksiyon.


///////////////////////////////////////////////////////////////////////////////
                            //Kapı Fonksiyonları//


void frontDoorClosed() {
  Serial.println("Kapı pul geçişine açılıyor.");
  dxlCom.setGoalPosition(frontDoor, 1002);
  while (!dxlCom.dxlDataReady())
    ;
} // Kapıyı kapatan fonksiyon.

void frontDoorOpen() {
  Serial.println("Kapı pul geçişine kapatılıyor.");
  dxlCom.setGoalPosition(frontDoor, 900);
  while (!dxlCom.dxlDataReady())
    ;
} // Kapıyı açan fonksiyon.

void backDoorClosed() {
  dxlCom.setGoalPosition(11,600);
  while (!dxlCom.dxlDataReady())
    ;
} // Kapıyı kapatan fonksiyon.

void backDoorOpen() {
  dxlCom.setGoalPosition(11, 510);
  while (!dxlCom.dxlDataReady())
    ;
} // Kapıyı açan fonksiyon.


///////////////////////////////////////////////////////////////////////////////
                        //Yönlendirici Fonksiyonları//


void redKabin(){
  dxlCom.setGoalPosition(rightHolder, rhRed);
  while (!dxlCom.dxlDataReady());
  delay(30);
  dxlCom.setGoalPosition(leftHolder, lhRed);
  while (!dxlCom.dxlDataReady());
} // Yönlendirici motorlarını kırmızı kabine yönlendiren (revize gerekli) fonksiyon.

void greenKabin(){
  dxlCom.setGoalPosition(rightHolder, rhGreen); 
  while (!dxlCom.dxlDataReady());
  delay(30);
  dxlCom.setGoalPosition(leftHolder, lhGreen);
  while (!dxlCom.dxlDataReady());
  Serial.println("Yeşil kabine yönlendirme yapılıyor.");
} // Yönlendirici motorlarını yeşil kabine yönlendiren fonksiyon.

void blueKabin(){
  dxlCom.setGoalPosition(rightHolder, rhBlue);
  while (!dxlCom.dxlDataReady());
  delay(50);
  dxlCom.setGoalPosition(leftHolder, lhBlue);
  while (!dxlCom.dxlDataReady());
  Serial.println("Mavi kabine yönlendirme yapılıyor.");
} // Yönlendirici motorlarını mavi kabine yönlendiren fonksiyon.


void redKabin2(){
  dxlCom.setGoalPosition(rightHolder, rhBlue);
  while (!dxlCom.dxlDataReady());
  delay(50);
  dxlCom.setGoalPosition(leftHolder, lhBlue);
  while (!dxlCom.dxlDataReady());
  Serial.println("Kırmızı kabine yönlendirme yapılıyor.");
}

void blueKabin2(){
  dxlCom.setGoalPosition(rightHolder, rhRed);
  while (!dxlCom.dxlDataReady());
  delay(30);
  dxlCom.setGoalPosition(leftHolder, lhRed);
  while (!dxlCom.dxlDataReady());
} 


void defaultKabin(){
  dxlCom.setGoalPosition(rightHolder, 446);
  while (!dxlCom.dxlDataReady());

  dxlCom.setGoalPosition(leftHolder, 668);
  while (!dxlCom.dxlDataReady());
}

///////////////////////////////////////////////////////////////////////////////
                        //MPU6050 ve Başlangıç Ayarları//


void tcsSetup(){
  // Renk sensörünün başlatılması.
  if (tcs.begin()) {
    Serial.println("Renk Sensörü Bulundu...");
  } else {
    Serial.println("TCS 34725 bağlantısı bulunamadı... bağlantınızı kontrol ediniz.");
    while (1) {}
  }
  // Gama düzeltmesinin yapılması. (teknik kısım)
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
} // Renk sensörünün başlatılması.

void robotDefault() {
  Serial.println("Robot başarıyla başlatıldı.");
  blueBagajClosed();    // Mavi renk bagajı intake pozisyonuna getiren fonksiyonun çağrılması.
  redBagajClosed();     // Kırmızı renk bagajı intake pozisyonuna getiren fonksiyonun çağrılması.
  greenBagajClosed();   // Yeşil renk bagajı intake pozisyonuna getiren fonksiyonun çağrılması.
  backDoorClosed();
  frontDoorClosed();
} // Bagajların açılması

void robotDefault2() {
  Serial.println("Robot başarıyla başlatıldı.");
  blueBagajClosed2();    // Mavi renk bagajı intake pozisyonuna getiren fonksiyonun çağrılması.
  redBagajClosed2();     // Kırmızı renk bagajı intake pozisyonuna getiren fonksiyonun çağrılması.
  greenBagajClosed();   // Yeşil renk bagajı intake pozisyonuna getiren fonksiyonun çağrılması.
  backDoorClosed();
  frontDoorClosed();
}


///////////////////////////////////////////////////////////////////////////////
                    //Sensör ile PID Hareket Fonksiyonu//


///////////////////////////////////////////////////////////////////////////////
                    //Bagaj Konumlandırma Fonksiyonları//


void redBagajClosed() {
    Serial.println("Kırmızı bagaj intake pozisyonuna getiriliyor.");
    dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb10_180);
    isRedBagajOpen = false;
} // Kırmızı bagajı intake pozisyonuna getiren fonksiyon.

void redBagajOpen() {
  redPulCounter = 0; // Kırmızı pulların boşaltılması.
  isRedBagajOpen = true;
  Serial.println("Kırmızı bagaj ve maksimuma ulaşmış pullar boşaltılıyor... Kırmızı pul sayısı :");
  dxlCom.setGoalPosition(bagajIndex[redBagajIndex], rb10_90);
} // Kırmızı bagajı başlangıç pozisyonuna getiren fonksiyon.




void redBagajClosed2() {
    Serial.println("Kırmızı bagaj intake pozisyonuna getiriliyor.");
    dxlCom.setGoalPosition(bagajIndex2[redBagajIndex2], rb8_180_2);
    isRedBagajOpen2 = false;
} // Kırmızı bagajı intake pozisyonuna getiren fonksiyon.

void redBagajOpen2() {
  redPulCounter2 = 0; // Kırmızı pulların boşaltılması.
  isRedBagajOpen2 = true;
  Serial.println("Kırmızı bagaj ve maksimuma ulaşmış pullar boşaltılıyor... Kırmızı pul sayısı :");
  dxlCom.setGoalPosition(bagajIndex2[redBagajIndex2], rb8_90_2);
}



void blueBagajClosed() {
  isBlueBagajOpen = false;
  Serial.println("Mavi bagaj intake pozisyonuna getiriliyor.");
  dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb8_180);
    
  
} // Mavi bagajı intake pozisyonuna getiren fonksiyon.
void blueBagajOpen() {
  isBlueBagajOpen = true;
  bluePulCounter = 0; // Mavi pulların boşaltılması.
  Serial.println("Mavi bagaj boşaltılıyor... Mavi pul sayısı :");
  dxlCom.setGoalPosition(bagajIndex[blueBagajIndex], bb8_90);
    
  
} // Mavi bagajı başlangıç pozisyonuna getiren fonksiyon.



void blueBagajClosed2() {
  isBlueBagajOpen2 = false;
  Serial.println("Mavi bagaj intake pozisyonuna getiriliyor.");
  dxlCom.setGoalPosition(bagajIndex2[blueBagajIndex2], bb10_180_2);
    
  
} // Mavi bagajı intake pozisyonuna getiren fonksiyon.
void blueBagajOpen2() {
  isBlueBagajOpen2 = true;
  bluePulCounter2 = 0; // Mavi pulların boşaltılması.
  Serial.println("Mavi bagaj boşaltılıyor... Mavi pul sayısı :");
  dxlCom.setGoalPosition(bagajIndex2[blueBagajIndex2], bb10_90_2);
} 




void greenBagajClosed() {
  isGreenBagajOpen = false;
  Serial.println("Yeşil bagaj intake pozisyonuna getiriliyor.");
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb9_180);
  
} // Yeşil bagajı intake pozisyonuna getiren fonksiyon.
void greenBagajOpen() {
  isGreenBagajOpen = true;
  greenPulCounter = 0; // Yeşil pulların boşaltılması.
  Serial.println("Yeşil bagaj boşaltılıyor... Yeşil pul sayısı :");
  Serial.print(greenPulCounter);
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb9_90);
  
} // Yeşil bagajı başlangıç pozisyonuna getiren fonksiyon.


void greenBagajClosed2() {
  isGreenBagajOpen2 = false;
  Serial.println("Yeşil bagaj intake pozisyonuna getiriliyor.");
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb9_180);
  
} // Yeşil bagajı intake pozisyonuna getiren fonksiyon.
void greenBagajOpen2() {
  isGreenBagajOpen2 = true;
  greenPulCounter2 = 0; // Yeşil pulların boşaltılması.
  Serial.println("Yeşil bagaj boşaltılıyor... Yeşil pul sayısı :");
  dxlCom.setGoalPosition(bagajIndex[greenBagajIndex], gb9_90);
  
}