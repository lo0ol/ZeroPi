#include "ZeroPi.h"

ZeroPi pi;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pi.timerSetup();
  pi.extInit(0, EXT_SERVO);
  pi.extWriteUs(0, 1350);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}
