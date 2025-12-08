// === 引脚定义（适配单电桥 L298 逻辑：1 PWM 速度脚 + 2 方向脚/每根杆） ===
// 物理接法（每根杆）：
//   IN1 → DIR1_PINS[i]，IN2 → DIR2_PINS[i]，IN3 → SPEED_PINS[i] (PWM)，IN4 → GND（常接地，简化为自由停）
// Mega2560 的 PWM 脚：2-13, 44-46（共 15 个）。这里为 14 根杆各占 1 个 PWM 速度脚。
const uint8_t NUM_ACTUATORS = 14;

// 速度脚（需 PWM）
const uint8_t SPEED_PINS[NUM_ACTUATORS] = {
  2, 3, 4, 5, 6, 7, 8,
  9, 10, 11, 12, 13, 44, 45
};

// 方向脚对（普通数字脚）：DIR1 高 / DIR2 低 = 前伸；DIR1 低 / DIR2 高 = 后缩
const uint8_t DIR1_PINS[NUM_ACTUATORS] = {
  22, 23, 24, 25, 26, 27, 28,
  29, 30, 31, 32, 33, 34, 35
};

const uint8_t DIR2_PINS[NUM_ACTUATORS] = {
  36, 37, 38, 39, 40, 41, 42,
  43, 48, 49, 50, 51, 52, 53
};

// R_EN 和 L_EN 在测试阶段直接接 5V，仍然保留管脚以便后续改为 Arduino 控制。
const uint8_t REN_PIN = 46;
const uint8_t LEN_PIN = 47;

const int DEFAULT_SPEED = 200;  // 0-255

// 预声明，避免 Arduino 预处理器生成的原型缺少类型定义
struct ParsedCommand;
bool parseCommand(const String &line, ParsedCommand &out);
void applyCommand(const ParsedCommand &cmd);

// 启动默认流程：先前伸 20s，再后缩 10s（速度 200），若收到指令则终止默认流程
enum DefaultStage { STAGE_FWD, STAGE_BACK, STAGE_DONE };
DefaultStage defaultStage = STAGE_FWD;
unsigned long defaultStageStartMs = 0;
bool defaultActive = true;

// 串口命令缓冲区
const uint8_t CMD_BUFFER_SIZE = 40;
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// === 基础动作（方向脚对: HIGH/LOW = 前伸，LOW/HIGH = 后缩） ===
void setDirection(uint8_t idx, bool forward) {
  digitalWrite(DIR1_PINS[idx], forward ? HIGH : LOW);
  digitalWrite(DIR2_PINS[idx], forward ? LOW : HIGH);
}

void moveForward(uint8_t idx, int speed) {
  setDirection(idx, true);
  analogWrite(SPEED_PINS[idx], speed);
}

void moveBackward(uint8_t idx, int speed) {
  setDirection(idx, false);
  analogWrite(SPEED_PINS[idx], speed);
}

void stopMotor(uint8_t idx) {
  analogWrite(SPEED_PINS[idx], 0);
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
// 低层命令（保留原有精细控制，便于后续扩展）：A<id>,<F|B|S>[,<0-255>] 或 ALL,<F|B|S>[,<0-255>]

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
      Serial.println("Cmd A: 1-2 F@200, 3-6 B@100, 7-14 stop");
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
      Serial.println("Cmd B: 1-2 B@200, 3-6 F@100, 7-14 stop");
      break;
    case 'S':
    default:
      stopAll();
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

void flushEnablePins() {
  // 测试阶段接 5V，这里保持高电平，后期如果不再直连可直接用这两个管脚。
  digitalWrite(REN_PIN, HIGH);
  digitalWrite(LEN_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    pinMode(SPEED_PINS[i], OUTPUT);
    pinMode(DIR1_PINS[i], OUTPUT);
    pinMode(DIR2_PINS[i], OUTPUT);
    stopMotor(i);
  }

  pinMode(REN_PIN, OUTPUT);
  pinMode(LEN_PIN, OUTPUT);
  flushEnablePins();

  Serial.println("=== 14-Actuator Controller Ready ===");
  Serial.println("High-level: A / B / S");
  Serial.println("Low-level: A<id>,<F|B|S>[,<0-255>]  e.g. A3,F,180");
  Serial.println("Broadcast: ALL,<F|B|S>[,<0-255>]  e.g. ALL,S");

  // 启动默认流程：先前伸 20s，再后缩 10s（若收到指令则打断）
  defaultStage = STAGE_FWD;
  defaultStageStartMs = millis();
  moveAllForward(200);
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
        // 收到指令后终止默认流程
        defaultActive = false;
        applyCommand(cmd);
      } else {
        Serial.print("Invalid cmd: ");
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

  // 默认流程的非阻塞调度
  if (defaultActive) {
    unsigned long elapsed = millis() - defaultStageStartMs;
    switch (defaultStage) {
      case STAGE_FWD:
        if (elapsed >= 20000UL) {  // 20s
          defaultStage = STAGE_BACK;
          defaultStageStartMs = millis();
          moveAllBackward(200);
          Serial.println("Default: switch to backward @200");
        }
        break;
      case STAGE_BACK:
        if (elapsed >= 10000UL) {  // 10s
          defaultStage = STAGE_DONE;
          defaultActive = false;
          stopAll();
          Serial.println("Default: done, all stopped");
        }
        break;
      case STAGE_DONE:
      default:
        break;
    }
  }
}
