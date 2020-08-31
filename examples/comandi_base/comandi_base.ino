#include "Easyino_Robot.h"
Easyino_Robot robot(facile);
void setup() {
  robot.begin();
}

void loop() {

  if (robot.riceve_qualcosa()) {
    robot.animazioneTagRiconosciuto();

    if (robot.codice_tessera() == AVANTI) {
      robot.vaiAvanti();
    }

    if (robot.codice_tessera() == INDIETRO) {
      robot.vaiIndietro();
    }

    if (robot.codice_tessera() == DESTRA) {
      robot.giraDestra();
    }

    if (robot.codice_tessera() == SINISTRA) {
      robot.giraSinistra();
    }

    if (robot.codice_tessera() == DESTRA_45) {
      robot.giraDestra(45);
    }
    
    if (robot.codice_tessera() == SINISTRA_45) {
      robot.giraSinistra(45);
    }
    
    if (robot.codice_tessera() == GIRO_COMPLETO) {
      robot.giraSinistra(360);
    }

    if (robot.codice_tessera() == LUCI_ANTERIORI) {
      robot.luciFrontali();
    }

    if (robot.codice_tessera() == LUCI_POSTERIORI) {
      robot.luciPosteriori();
    }

    if (robot.codice_tessera() == FRECCIA_DESTRA) {
      robot.accendiFrecciaDestra();
    }

    if (robot.codice_tessera() == FRECCIA_SINISTRA) {
      robot.accendiFrecciaSinistra();
    }
  }
}
