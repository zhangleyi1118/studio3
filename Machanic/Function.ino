// === 引脚定义 ===
const int RPWM_PIN = 5;
const int LPWM_PIN = 6;
const int REN_PIN  = 7;
const int LEN_PIN  = 8;

// === 默认速度 ===
const int DEFAULT_SPEED = 255;

// ========================================
// 函数：向前伸展
// ========================================
void moveForward(int speed = DEFAULT_SPEED) {
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);
  analogWrite(RPWM_PIN, speed);
  analogWrite(LPWM_PIN, 0);
  Serial.print("Moving Forward, Speed: ");
  Serial.println(speed);
}

// ========================================
// 函数：向后收缩
// ========================================
void moveBackward(int speed = DEFAULT_SPEED) {
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, speed);
  Serial.print("Moving Backward, Speed: ");
  Serial.println(speed);
}

// ========================================
// 函数：停止
// ========================================
void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  Serial.println("Motor Stopped");
}

// ========================================
// 初始化
// ========================================
void setup() {
  Serial.begin(9600);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(LEN_PIN, OUTPUT);

  // 初始全部关闭
  digitalWrite(REN_PIN, LOW);
  digitalWrite(LEN_PIN, LOW);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  Serial.println("=== Actuator Control Ready ===");
  Serial.println("Commands: F=Forward, B=Backward, S=Stop");
}

// ========================================
// 主循环：监听串口指令
// ========================================
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // 处理指令
    switch (command) {
      case 'F':
      case 'f':
        moveForward();
        break;
        
      case 'B':
      case 'b':
        moveBackward();
        break;
        
      case 'S':
      case 's':
        stopMotor();
        break;
        
      default:
        // 忽略无效字符（如换行符等）
        break;
    }
  }
}
