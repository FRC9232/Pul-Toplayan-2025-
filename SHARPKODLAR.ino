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

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(200);   

  // Her sensörden mesafeyi oku
  int dis_onSol = ir_onSol.distance();
  int dis_onSag = ir_onSag.distance();
  int dis_sol   = ir_sol.distance();
  int dis_sag   = ir_sag.distance();

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
