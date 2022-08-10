#include <obniz.h>
void setup() {
  Serial.begin(115200);
  obniz.start();
  obniz.pinReserve(2);
  pinMode(2, OUTPUT);
}
void loop() {
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}
