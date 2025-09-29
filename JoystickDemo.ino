#define VRX_PIN 15
#define VRY_PIN 16
#define SW_PIN  17

void setup() {
  Serial.begin(115200);
  pinMode(SW_PIN, INPUT_PULLUP);
}

void loop() {
  int x, y, sw;
  float x_volt, y_volt;
  x = analogRead(VRX_PIN);
  y = analogRead(VRY_PIN);
  sw = digitalRead(SW_PIN);

  x_volt = ((x * 3.3) / 4095);
  y_volt = ((y * 3.3) / 4095);
  
  Serial.print("X Voltage = ");
  Serial.print(x_volt);
  Serial.print("\tY Voltage = ");
  Serial.print(y_volt);

  if (sw == LOW) {
    Serial.println("\tButton PRESSED");
  } else {
    Serial.println("\tButton released");
  }
  delay(100);
}
