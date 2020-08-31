#include "Easyino_Robot.h"
Easyino_Robot robot(facile);
void setup() {
  robot.begin();
}

void loop() {

  if (robot.riceve_qualcosa()) {
    robot.animazioneTagRiconosciuto();

     if (robot.codice_tessera() == AVANTI) {
      robot.luciFrontali();
      robot.vaiAvanti();
    }

     if (robot.codice_tessera() == INDIETRO) {
      robot.luciPosteriori();
      robot.vaiIndietro();
    }

    if (robot.codice_tessera() == DESTRA) {
      robot.accendiFrecciaDestra();
      robot.giraDestra();
    }

     if (robot.codice_tessera() == SINISTRA) {
      robot.accendiFrecciaSinistra();
      robot.giraSinistra();
    }

  }
}
