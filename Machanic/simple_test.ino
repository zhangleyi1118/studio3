// 简化测试代码 - 测试D22引脚是否能控制
void setup() {
  Serial.begin(115200);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  
  Serial.println("=== Simple Test Ready ===");
  Serial.println("Commands: 1=D22 HIGH, 0=D22 LOW");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == '1') {
      digitalWrite(22, HIGH);
      digitalWrite(23, LOW);
      Serial.println("D22=HIGH, D23=LOW");
    } 
    else if (cmd == '0') {
      digitalWrite(22, LOW);
      digitalWrite(23, LOW);
      Serial.println("D22=LOW, D23=LOW");
    }
    else if (cmd == '2') {
      digitalWrite(22, LOW);
      digitalWrite(23, HIGH);
      Serial.println("D22=LOW, D23=HIGH");
    }
  }
}
