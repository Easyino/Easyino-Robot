#include <Easyino_Robot.h>
Easyino_Robot robot(facile);
void setup() {
  robot.begin();
}

void loop() {

  if (robot.riceve_qualcosa()) {
    robot.animazioneTagRiconosciuto();

     if (robot.codice_tessera() == AVANTI) {
      robot.luci_frontali();
      robot.vaiAvanti(100);
    }

     if (robot.codice_tessera() == INDIETRO) {
      robot.luci_posteriori();
      robot.vaiIndietro(100);
    }

    if (robot.codice_tessera() == DESTRA) {
      robot.accendiFrecciaDestra();
      robot.giraDestra(90);
    }

     if (robot.codice_tessera() == SINISTRA) {
      robot.accendiFrecciaSinistra();
      robot.giraSinistra(90);
    }

  }
}
