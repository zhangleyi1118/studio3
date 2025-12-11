// === 引脚定义（BTS7960 双H桥驱动：2个PWM + 2个使能/每根杆） ===
// BTS7960 正确工作逻辑（实测验证）：
//   ✅ R_EN和L_EN都保持HIGH（两个使能都开启）
//   正转：RPWM=PWM, LPWM=0
//   反转：RPWM=0, LPWM=PWM
//   停止：RPWM=0, LPWM=0
//   ⚠️ 禁止：RPWM和LPWM同时有PWM信号（会短路烧毁！）
//   ⚠️ 注意：如果EN互斥使用会导致电机两端电压差消失，无法产生扭矩
//
// 物理接法（每根杆）：
//   R_EN → R_EN_PINS[i]（右侧使能，正转时=HIGH）
//   L_EN → L_EN_PINS[i]（左侧使能，反转时=HIGH）
//   RPWM → RPWM_PINS[i]（右侧PWM，正转速度控制）
//   LPWM → LPWM_PINS[i]（左侧PWM，反转速度控制）
//
// 分组方案（因Mega2560 PWM引脚有限）：
//   组1（杆1-2）：RPWM=D2, LPWM=D3
//   组2（杆3-6）：RPWM=D4, LPWM=D5
//   组3（杆7-14）：RPWM=D6, LPWM=D7
const uint8_t NUM_ACTUATORS = 14;

// RPWM引脚（正转PWM，分组共用）
const uint8_t RPWM_PINS[NUM_ACTUATORS] = {
  2, 2, 4, 4, 4, 4, 6,      // 杆1-2用D2, 杆3-6用D4, 杆7用D6
  6, 6, 6, 6, 6, 6, 6       // 杆8-14用D6
};

// LPWM引脚（反转PWM，分组共用）
const uint8_t LPWM_PINS[NUM_ACTUATORS] = {
  3, 3, 5, 5, 5, 5, 7,      // 杆1-2用D3, 杆3-6用D5, 杆7用D7
  7, 7, 7, 7, 7, 7, 7       // 杆8-14用D7
};

// R_EN引脚（右侧使能，普通数字脚）
const uint8_t R_EN_PINS[NUM_ACTUATORS] = {
  22, 24, 26, 28, 30, 32, 34,
  36, 38, 40, 42, 46, 48, 50
};

// L_EN引脚（左侧使能，普通数字脚）
const uint8_t L_EN_PINS[NUM_ACTUATORS] = {
  23, 25, 27, 29, 31, 33, 35,
  37, 39, 41, 43, 47, 49, 51
};

// TB6600 步进电机（新增第 15 路执行器，不占 PWM）。
// 说明：原有 14 路已占用 D51~D53，因此避免冲突，使用 A0-A2（D54-D56）。
// 若仍想用 D51/D52/D53，需要把对应杆的 DIR2 线移到其他空闲脚。
const uint8_t STEPPER_STEP_PIN = 54;  // A0
const uint8_t STEPPER_DIR_PIN  = 55;  // A1
const uint8_t STEPPER_ENA_PIN  = 56;  // A2，可选未接

// 步进电机速度：0-600 RPM
// 600 RPM = 10转/秒，假设200步/转 = 2000步/秒
// 脉冲翻转间隔 = 1/(2000步/秒 × 2次翻转/步) = 250us
// 可调范围：83us (最快1800RPM) ~ 500us (最慢300RPM)
const unsigned long STEPPER_PULSE_INTERVAL_US = 500;  // 300 RPM

bool stepperRunning = false;
bool stepperLeft = true;  // true = 左（DIR 低），false = 右（DIR 高）
unsigned long lastStepperToggleUs = 0;
bool stepperStepLevel = LOW;

const int DEFAULT_SPEED = 200;  // 0-255

// 预声明，避免 Arduino 预处理器生成的原型缺少类型定义
struct ParsedCommand;
bool parseCommand(const String &line, ParsedCommand &out);
void applyCommand(const ParsedCommand &cmd);

// 串口命令缓冲区
const uint8_t CMD_BUFFER_SIZE = 40;
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// === 基础动作（BTS7960控制：R_EN/L_EN都保持HIGH，只用RPWM/LPWM控制方向） ===
// BTS7960正确用法：两个使能都HIGH，方向由PWM控制
void enableMotor(uint8_t idx) {
  digitalWrite(R_EN_PINS[idx], HIGH);
  digitalWrite(L_EN_PINS[idx], HIGH);
}

// === TB6600 步进控制（非阻塞方波脉冲） ===
void enableStepper(bool enable) {
  // 若 ENA 未接，这里保持低电平也无副作用。
  digitalWrite(STEPPER_ENA_PIN, enable ? LOW : HIGH);  // TB6600: ENA- 低 = 使能
}

void stopStepper() {
  stepperRunning = false;
  stepperStepLevel = LOW;
  digitalWrite(STEPPER_STEP_PIN, LOW);
}

void startStepper(bool left) {
  stepperLeft = left;
  digitalWrite(STEPPER_DIR_PIN, left ? LOW : HIGH);
  stepperRunning = true;
  stepperStepLevel = LOW;
  lastStepperToggleUs = micros();
}

void moveForward(uint8_t idx, int speed) {
  // BTS7960正确用法：R_EN/L_EN都HIGH，方向由PWM控制
  digitalWrite(R_EN_PINS[idx], HIGH);
  digitalWrite(L_EN_PINS[idx], HIGH);
  // 正转：RPWM=PWM, LPWM=0（确保互补，避免短路）
  analogWrite(RPWM_PINS[idx], speed);
  analogWrite(LPWM_PINS[idx], 0);
}

void moveBackward(uint8_t idx, int speed) {
  // BTS7960正确用法：R_EN/L_EN都HIGH，方向由PWM控制
  digitalWrite(R_EN_PINS[idx], HIGH);
  digitalWrite(L_EN_PINS[idx], HIGH);
  // 反转：RPWM=0, LPWM=PWM（确保互补，避免短路）
  analogWrite(RPWM_PINS[idx], 0);
  analogWrite(LPWM_PINS[idx], speed);
}

void stopMotor(uint8_t idx) {
  // 停止：保持EN=HIGH，PWM都归零（惯性滑行）
  // 或者都LOW完全关闭（主动刹车）
  digitalWrite(R_EN_PINS[idx], HIGH);
  digitalWrite(L_EN_PINS[idx], HIGH);
  analogWrite(RPWM_PINS[idx], 0);
  analogWrite(LPWM_PINS[idx], 0);
}

void stopAll() {
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    stopMotor(i);
  }
}

void moveAllForward(int speed) {
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    moveForward(i, speed);
  }
}

void moveAllBackward(int speed) {
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    moveBackward(i, speed);
  }
}

// === 命令处理 ===
// 高层命令：
//   'A' : 1-2 前伸@200；3-6 后缩@100；7-14 保持停止
//   'B' : 与 A 相反
//   'S' : 全部停止
// 低层命令（保留原有精细控制，便于后续扩展）：<id>,<F|B|S>[,<0-255>] 或 ALL,<F|B|S>[,<0-255>]
// 示例：1,F,255  (1号电机前进255速度)  或  ALL,S  (全部停止)

struct ParsedCommand {
  bool isHighLevel;   // true 表示单字母高层命令
  bool isBroadcast;
  int targetIndex;    // 0-based when not broadcast
  char direction;     // 'F' forward, 'B' backward, 'S' stop
  int speed;          // 0-255
  char rawHighLevel;  // 保存高层指令字符
};

bool parseCommand(const String &line, ParsedCommand &out) {
  // 高层单字符
  if (line.length() == 1) {
    char c = (char)toupper(line[0]);
    if (c == 'A' || c == 'B' || c == 'S') {
      out.isHighLevel = true;
      out.rawHighLevel = c;
      return true;
    }
  }

  int firstComma = line.indexOf(',');
  if (firstComma == -1) {
    return false;
  }

  String target = line.substring(0, firstComma);
  String rest = line.substring(firstComma + 1);

  int secondComma = rest.indexOf(',');
  String dirPart = (secondComma == -1) ? rest : rest.substring(0, secondComma);
  String speedPart = (secondComma == -1) ? "" : rest.substring(secondComma + 1);

  dirPart.trim();
  speedPart.trim();
  target.trim();

  char dir = dirPart.length() ? (char)toupper(dirPart[0]) : 'S';
  if (dir != 'F' && dir != 'B' && dir != 'S') {
    return false;
  }

  int spd = speedPart.length() ? speedPart.toInt() : DEFAULT_SPEED;
  spd = constrain(spd, 0, 255);

  if (target.equalsIgnoreCase("ALL")) {
    out.isBroadcast = true;
    out.targetIndex = -1;
  } else {
    int idx = target.toInt() - 1;  // 人类友好的 1 起始
    if (idx < 0 || idx >= NUM_ACTUATORS) {
      return false;
    }
    out.isBroadcast = false;
    out.targetIndex = idx;
  }

  out.isHighLevel = false;
  out.direction = dir;
  out.speed = spd;
  return true;
}

void applyHighLevel(char c) {
  switch (c) {
    case 'A':
      moveForward(0, 200);
      moveForward(1, 200);
      moveBackward(2, 100);
      moveBackward(3, 100);
      moveBackward(4, 100);
      moveBackward(5, 100);
      for (uint8_t i = 6; i < NUM_ACTUATORS; i++) {
        stopMotor(i);
      }
      startStepper(true);  // 左转
      Serial.println("Cmd A: 1-2 F@200, 3-6 B@100, 7-14 stop + Stepper LEFT");
      break;
    case 'B':
      moveBackward(0, 200);
      moveBackward(1, 200);
      moveForward(2, 100);
      moveForward(3, 100);
      moveForward(4, 100);
      moveForward(5, 100);
      for (uint8_t i = 6; i < NUM_ACTUATORS; i++) {
        stopMotor(i);
      }
      startStepper(false);  // 右转
      Serial.println("Cmd B: 1-2 B@200, 3-6 F@100, 7-14 stop + Stepper RIGHT");
      break;
    case 'S':
    default:
      stopAll();
      stopStepper();
      Serial.println("Cmd S: all stop");
      break;
  }
}

void applyCommand(const ParsedCommand &cmd) {
  if (cmd.isHighLevel) {
    applyHighLevel(cmd.rawHighLevel);
    return;
  }

  auto driveOne = [&](uint8_t idx) {
    switch (cmd.direction) {
      case 'F':
        moveForward(idx, cmd.speed);
        Serial.print("Actuator ");
        Serial.print(idx + 1);
        Serial.print(" -> Forward @");
        Serial.println(cmd.speed);
        break;
      case 'B':
        moveBackward(idx, cmd.speed);
        Serial.print("Actuator ");
        Serial.print(idx + 1);
        Serial.print(" -> Backward @");
        Serial.println(cmd.speed);
        break;
      case 'S':
      default:
        stopMotor(idx);
        Serial.print("Actuator ");
        Serial.print(idx + 1);
        Serial.println(" -> Stop");
        break;
    }
  };

  if (cmd.isBroadcast) {
    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
      driveOne(i);
    }
  } else {
    driveOne(cmd.targetIndex);
  }
}

void setup() {
  Serial.begin(115200);

  // 初始化所有电机控制引脚
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    pinMode(R_EN_PINS[i], OUTPUT);
    pinMode(L_EN_PINS[i], OUTPUT);
    pinMode(RPWM_PINS[i], OUTPUT);
    pinMode(LPWM_PINS[i], OUTPUT);
    stopMotor(i);  // 初始化为安全停止状态
  }

  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENA_PIN, OUTPUT);
  enableStepper(true);
  stopStepper();

  Serial.println("=== 14-Actuator BTS7960 Controller Ready ===");
  Serial.println("High-level: A / B / S");
  Serial.println("Low-level: <id>,<F|B|S>[,<0-255>]  e.g. 3,F,180");
  Serial.println("Broadcast: ALL,<F|B|S>[,<0-255>]  e.g. ALL,S");
  Serial.println("Stepper: A=LEFT, B=RIGHT, S=STOP (TB6600)");
  Serial.println("Driver: BTS7960 Double H-Bridge");
  Serial.println("Ready to receive commands...");
}

void loop() {
  // 非阻塞读取串口，及时响应
  while (Serial.available()) {
    char incoming = Serial.read();

    if (incoming == '\n' || incoming == '\r') {
      if (cmdIndex == 0) {
        continue;  // 忽略空行
      }

      cmdBuffer[cmdIndex] = '\0';
      String line = String(cmdBuffer);
      cmdIndex = 0;  // 重置缓冲区

      ParsedCommand cmd;
      if (parseCommand(line, cmd)) {
        applyCommand(cmd);
      } else {
        Serial.print("Invalid command: ");
        Serial.println(line);
      }
    } else {
      if (cmdIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdIndex++] = incoming;
      } else {
        // 缓冲区溢出保护，丢弃并重置
        cmdIndex = 0;
      }
    }
  }

  // 步进电机脉冲生成（非阻塞）
  if (stepperRunning) {
    unsigned long nowUs = micros();
    if (nowUs - lastStepperToggleUs >= STEPPER_PULSE_INTERVAL_US) {
      lastStepperToggleUs = nowUs;
      stepperStepLevel = !stepperStepLevel;
      digitalWrite(STEPPER_STEP_PIN, stepperStepLevel);
    }
  }
}