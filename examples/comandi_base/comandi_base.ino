#include "Easyino_Robot.h"
Easyino_Robot robot(false);
void setup() {
  robot.begin();
}

void loop() {

  if (robot.riceve_qualcosa()) {
    Serial.println(robot.codice_tessera());
    robot.animazioneTagRiconosciuto();
    if (robot.codice_tessera() == DESTRA) {
      robot.accendiFrecciaDestra();
      robot.giraDestra(90);
    }
    else if (robot.codice_tessera() == SINISTRA) {
      robot.accendiFrecciaSinistra();
      robot.giraSinistra(90);
    }
    else if (robot.codice_tessera() == AVANTI) {
      robot.luci_frontali();
      robot.vaiAvanti(100);
    }
    else if (robot.codice_tessera() == INDIETRO) {
      robot.luci_posteriori();
      robot.vaiIndietro(100);
    }
    else if (robot.codice_tessera() == 229) {
robot.taraPiuDestra();
    }
        else if (robot.codice_tessera() == 71) {
robot.taraPiuSinistra();
    }
    
    else if (robot.codice_tessera() == 5) {
      robot.avanti(100);
      robot.sinistra(90); 
      robot.avanti(33);
      robot.destra(90);
      robot.avanti(50);
      robot.sinistra(90);
      robot.avanti(283);
    }
  }
  Serial.print("\n kdx: ");
  Serial.print(robot.kdx);
  Serial.print("--- ksx: ");
  Serial.print(robot.ksx);
}
