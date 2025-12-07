void setup() {
  Serial.begin(115200);
  pinMode(6, INPUT_PULLUP);   // 使用内部上拉
  Serial.println("Testing switch on GPIO6...");
}

void loop() {
  int state = digitalRead(6);
  Serial.println(state == LOW ? "Pressed" : "Released");
  delay(200); // 0.2 秒打印一次，避免太快
}
