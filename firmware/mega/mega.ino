void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("READY");

}
void loop() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command == "PING") {
    Serial.println("PONG");
  }
  else if (command == "LED ON") {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("OK");
  }
  else if (command == "LED OFF") {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("OK");
  }
  else {
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}