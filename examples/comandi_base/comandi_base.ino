#include <Easyino_Robot.h>
Easyino_Robot robot(true);
void setup() {
  robot.begin();
  digitalWrite(3, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(A4, LOW);
  robot.tag_riconosciuto();
}

void loop() {
  if (robot.riceve_qualcosa()) {
    Serial.println(robot.codice_tessera());
    robot.tag_riconosciuto();
    if (robot.codice_tessera() == AVANTI) {
      robot.accendiFrecciaSinistra();
      robot.giraDestra(90);
    }
    else if (robot.codice_tessera() == 2) {
      robot.accendiFrecciaSinistra();
      robot.giraSinistra(90);
    }
    else if (robot.codice_tessera() == 3) {
      robot.luci_frontali();
      robot.vaiAvanti(100);
    }
    else if (robot.codice_tessera() == 4) {
      robot.luci_posteriori();
      robot.vaiIndietro(100);
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
}