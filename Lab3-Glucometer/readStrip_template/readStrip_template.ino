#include <Wire.h>

float V1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Initializing...");

  pinMode(A2, INPUT);
}

void loop() {
  
  // Read V1 and convert to voltage (3 pts)
  V1 = analogRead(A2);

  // convert bits to voltage
  V1 = (V1 * 3.3) / 1023;

  Serial.println(V1);

  delay(10);

}
