#include"Easyino_Robot.h"
Easyino_Robot robot(true);
void setup() {
  robot.begin();
}


void loop() {
  if (robot.riceveQualcosa()) {

    Serial.println(robot.codice_tessera());


  }
}
