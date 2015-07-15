#include "ZeroPi.h"

ZeroPi pi;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pi.slotSetup(0, SLOT_STEPPER);
  pi.stepperSetResolution(0, 16);
  pi.stepperEnable(0, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello world");
  delay(1000);
  pi.stepperMove(0,1);
}
