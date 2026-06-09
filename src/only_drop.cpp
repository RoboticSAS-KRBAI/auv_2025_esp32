// #include <Arduino.h>
// #define CONTROL_PIN 5

// void setup() {
//   Serial.begin(115200);

//   pinMode(CONTROL_PIN, OUTPUT);
//   digitalWrite(CONTROL_PIN, LOW); // default OFF

//   Serial.println("READY");
// }

// void loop() {
//   if (Serial.available()) {
//     char c = Serial.read();
//     Serial.print("Received: ");
//     Serial.println(c);

//     if (c == '1') {
//       digitalWrite(CONTROL_PIN, HIGH);
//       Serial.println("PIN HIGH");
//     } 
//     else if (c == '0') {
//       digitalWrite(CONTROL_PIN, LOW);
//       Serial.println("PIN LOW");
//     }
//   }
// }