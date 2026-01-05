// === 引脚定义 ===
const int RPWM_PIN = 5;
const int LPWM_PIN = 6;
const int REN_PIN  = 7;
const int LEN_PIN  = 8;

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

  Serial.println("Motor ready.");
}

void loop() {

  // === 正转 20 秒 ===
  Serial.println("Forward 20s");
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);
  analogWrite(RPWM_PIN, 255);   // 100% 正转
  analogWrite(LPWM_PIN, 0);
  delay(20000);

  // === 停止 1 秒 ===
  Serial.println("Stop 1s");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(1000);

  // === 反转 20 秒 ===
  Serial.println("Backward 20s");
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 255);   // 100% 反转
  delay(20000);

  // === 停止 1 秒 ===
  Serial.println("Stop 1s");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(1000);
}
