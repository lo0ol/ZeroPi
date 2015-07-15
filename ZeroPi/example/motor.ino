#include "ZeroPi.h"

ZeroPi pi;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SET_OUTPUT(A0);
  pi.begin();
  pi.motorInit(0);
  pi.motorRun(0, 50, -200);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}
