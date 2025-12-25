// === 多组件同步控制系统 ===
// 硬件组件：
//   - 组1伸缩杆（1-6）：共用PWM D2/D3，独立EN引脚 D22-D33
//   - 组2伸缩杆（7-14）：共用PWM D4/D5，独立EN引脚 D34-D51
//   - 步进电机滑台：STEP=D54, DIR=D55, ENA=D56
//   - 舵机MG996R：PWM引脚 D9
//
// 进度定义（100%基准）：统一15秒作为100%时间
//   - 伸缩杆：10cm @ 6mm/s → 统一到15秒
//   - 滑台：100cm @ 60mm/s → 统一到15秒
//   - 舵机：180° @ 10°/s → 统一到15秒（转不到底没关系）

#include <Servo.h>

// === 引脚定义 ===
const uint8_t NUM_ACTUATORS = 14;

// 分组方案：
//   组1（杆1-6）：RPWM=D2, LPWM=D3
//   组2（杆7-14）：RPWM=D4, LPWM=D5

// RPWM引脚（正转PWM，分组共用）
const uint8_t RPWM_PINS[NUM_ACTUATORS] = {
  2, 2, 2, 2, 2, 2,        // 组1：杆1-6共用D2
  4, 4, 4, 4, 4, 4, 4, 4   // 组2：杆7-14共用D4
};

// LPWM引脚（反转PWM，分组共用）
const uint8_t LPWM_PINS[NUM_ACTUATORS] = {
  3, 3, 3, 3, 3, 3,        // 组1：杆1-6共用D3
  5, 5, 5, 5, 5, 5, 5, 5   // 组2：杆7-14共用D5
};

// R_EN引脚（右侧使能，每根杆独立）
const uint8_t R_EN_PINS[NUM_ACTUATORS] = {
  22, 24, 26, 28, 30, 32,  // 组1：杆1-6的R_EN
  34, 36, 38, 40, 42, 44, 46, 48  // 组2：杆7-14的R_EN
};

// L_EN引脚（左侧使能，每根杆独立）
const uint8_t L_EN_PINS[NUM_ACTUATORS] = {
  23, 25, 27, 29, 31, 33,  // 组1：杆1-6的L_EN
  35, 37, 39, 41, 43, 45, 47, 49  // 组2：杆7-14的L_EN
};

// 步进电机引脚
const uint8_t STEPPER_STEP_PIN = 54;  // A0
const uint8_t STEPPER_DIR_PIN  = 55;  // A1
const uint8_t STEPPER_ENA_PIN  = 56;  // A2

// 舵机引脚
const uint8_t SERVO_PIN = 9;

// === 步进电机参数 ===
const unsigned long STEPPER_PULSE_INTERVAL_US = 500;  // 300 RPM
bool stepperRunning = false;
bool stepperLeft = true;
unsigned long lastStepperToggleUs = 0;
bool stepperStepLevel = LOW;
long stepperPosition = 0;  // 位置跟踪（右为正，左为负）

// === 舵机对象 ===
Servo servo;

// === 状态管理系统 ===
struct MotionState {
  bool isRunning;
  unsigned long startTime;
  unsigned long duration;  // 毫秒，统一为 15000ms * percent / 100
  char direction;  // 'F' forward, 'B' backward, 'L' left, 'R' right
  int percent;
};

enum MainState {
  IDLE,
  RUNNING,
  STOPPING,
  RESETTING
};

enum InitState {
  INIT_IDLE,
  INIT_ACTUATORS_FORWARD,
  INIT_ACTUATORS_BACKWARD,
  INIT_COMPLETE
};

struct SystemState {
  MotionState group1;  // 组1伸缩杆
  MotionState group2;  // 组2伸缩杆
  MotionState stepper; // 步进电机
  MotionState servo;   // 舵机
  unsigned long lastCommandTime;
  bool inCooldown;  // 30s控制间隙
  MainState mainState;
  unsigned long stateStartTime;
  int lastPercent;  // 记录上次百分比，用于复位
  
  // 初始状态恢复状态机
  InitState initState;
  unsigned long initStartTime;
  bool initActuators;
  bool initStepper;
  bool initServo;
};

SystemState systemState;

// === 命令解析 ===
struct ParsedCommand {
  bool isMainMode;      // true = 主模式 (f, 20), false = 调试模式
  bool isStop;          // true = 强制停止命令 's'
  bool isStart;         // true = START命令
  bool startActuators;  // START,ACTUATORS
  bool startStepper;    // START,STEPPER
  bool startServo;      // START,SERVO
  bool startAll;        // START,ALL
  char mainDirection;   // 'F' or 'B' for main mode
  int mainPercent;      // 0-100 for main mode
  
  // 调试模式字段
  bool isGroup1;
  bool isGroup2;
  bool isStepper;
  bool isServo;
  bool isAll;
  char debugDirection;  // 'F', 'B', 'L', 'R'
  int debugPercent;     // 0-100
  int servoTargetAngle; // 舵机目标角度（0-180）
};

const int DEFAULT_SPEED = 200;  // 0-255
const uint8_t CMD_BUFFER_SIZE = 40;
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// === 时间计算 ===
unsigned long calculateDuration(int percent) {
  // 统一时间基准：15秒 = 100%
  return (unsigned long)(15000 * percent / 100);
}

// === PWM冲突防护：组控制函数（重要！） ===
void stopGroup(uint8_t group) {
  uint8_t start, end;
  if (group == 1) {
    start = 0; end = 5;  // 杆1-6（索引0-5）
  } else if (group == 2) {
    start = 6; end = 13;  // 杆7-14（索引6-13）
  } else {
    return;
  }
  
  // 先停止所有EN，确保安全
  for (uint8_t i = start; i <= end; i++) {
    digitalWrite(R_EN_PINS[i], LOW);
    digitalWrite(L_EN_PINS[i], LOW);
  }
  
  // 确保PWM完全清零
  analogWrite(RPWM_PINS[start], 0);
  analogWrite(LPWM_PINS[start], 0);
  
  // 更新状态
  if (group == 1) {
    systemState.group1.isRunning = false;
  } else {
    systemState.group2.isRunning = false;
  }
}

void moveGroupForward(uint8_t group, int speed) {
  uint8_t start, end;
  if (group == 1) {
    start = 0; end = 5;
  } else if (group == 2) {
    start = 6; end = 13;
  } else {
    return;
  }
  
  // 先停止同组内所有杆（确保PWM清零）
  stopGroup(group);
  delay(10);  // 短暂延迟确保PWM完全清零
  
  // 然后统一启动
  for (uint8_t i = start; i <= end; i++) {
    digitalWrite(R_EN_PINS[i], HIGH);
    digitalWrite(L_EN_PINS[i], HIGH);
  }
  
  // 设置PWM（确保RPWM和LPWM互斥）
  analogWrite(RPWM_PINS[start], speed);
  analogWrite(LPWM_PINS[start], 0);  // 确保LPWM=0
  
  // 更新状态
  if (group == 1) {
    systemState.group1.isRunning = true;
    systemState.group1.direction = 'F';
  } else {
    systemState.group2.isRunning = true;
    systemState.group2.direction = 'F';
  }
}

void moveGroupBackward(uint8_t group, int speed) {
  uint8_t start, end;
  if (group == 1) {
    start = 0; end = 5;
  } else if (group == 2) {
    start = 6; end = 13;
  } else {
    return;
  }
  
  // 先停止同组内所有杆（确保PWM清零）
  stopGroup(group);
  delay(10);  // 短暂延迟确保PWM完全清零
  
  // 然后统一启动
  for (uint8_t i = start; i <= end; i++) {
    digitalWrite(R_EN_PINS[i], HIGH);
    digitalWrite(L_EN_PINS[i], HIGH);
  }
  
  // 设置PWM（确保RPWM和LPWM互斥）
  analogWrite(RPWM_PINS[start], 0);  // 确保RPWM=0
  analogWrite(LPWM_PINS[start], speed);
  
  // 更新状态
  if (group == 1) {
    systemState.group1.isRunning = true;
    systemState.group1.direction = 'B';
  } else {
    systemState.group2.isRunning = true;
    systemState.group2.direction = 'B';
  }
}

// === 步进电机控制 ===
void enableStepper(bool enable) {
  digitalWrite(STEPPER_ENA_PIN, enable ? LOW : HIGH);  // TB6600: ENA- 低 = 使能
}

void stopStepper() {
  stepperRunning = false;
  stepperStepLevel = LOW;
  digitalWrite(STEPPER_STEP_PIN, LOW);
  systemState.stepper.isRunning = false;
}

void startStepper(bool left, int percent) {
  stepperLeft = left;
  digitalWrite(STEPPER_DIR_PIN, left ? LOW : HIGH);
  stepperRunning = true;
  stepperStepLevel = LOW;
  lastStepperToggleUs = micros();
  
  systemState.stepper.isRunning = true;
  systemState.stepper.direction = left ? 'L' : 'R';
  systemState.stepper.percent = percent;
  systemState.stepper.startTime = millis();
  systemState.stepper.duration = calculateDuration(percent);
}

// === 舵机控制 ===
void setServoAngle(int targetAngle, int percent) {
  // 按百分比计算实际角度（转不到底没关系）
  int actualAngle = (targetAngle * percent) / 100;
  actualAngle = constrain(actualAngle, 0, 180);
  servo.write(actualAngle);
  
  systemState.servo.isRunning = true;
  systemState.servo.startTime = millis();
  systemState.servo.duration = calculateDuration(percent);
  systemState.servo.percent = percent;
}

// === 主工作模式状态机 ===
void startMainMotion(int percent, char direction) {
  systemState.lastPercent = percent;
  unsigned long duration = calculateDuration(percent);
  unsigned long now = millis();
  
  // 启动所有组件
  int speed = 255;  // 100%速度
  
  if (direction == 'F') {
    moveGroupForward(1, speed);
    moveGroupForward(2, speed);
    startStepper(false, percent);  // 右移
    setServoAngle(180, percent);   // 逆时针转
  } else {  // 'B'
    moveGroupBackward(1, speed);
    moveGroupBackward(2, speed);
    startStepper(true, percent);   // 左移
    setServoAngle(0, percent);     // 顺时针转（回到0度）
  }
  
  // 更新状态
  systemState.group1.isRunning = true;
  systemState.group1.startTime = now;
  systemState.group1.duration = duration;
  systemState.group1.direction = direction;
  systemState.group1.percent = percent;
  
  systemState.group2.isRunning = true;
  systemState.group2.startTime = now;
  systemState.group2.duration = duration;
  systemState.group2.direction = direction;
  systemState.group2.percent = percent;
  
  systemState.mainState = RUNNING;
  systemState.stateStartTime = now;
  
  Serial.print("Main motion started: ");
  Serial.print(direction);
  Serial.print(", ");
  Serial.print(percent);
  Serial.println("%");
}

void stopAllMotion() {
  stopGroup(1);
  stopGroup(2);
  stopStepper();
  
  systemState.group1.isRunning = false;
  systemState.group2.isRunning = false;
  systemState.stepper.isRunning = false;
  systemState.servo.isRunning = false;
  
  Serial.println("All motion stopped");
}

void resetMainMotion() {
  // 反向复位
  char reverseDir = (systemState.group1.direction == 'F') ? 'B' : 'F';
  startMainMotion(systemState.lastPercent, reverseDir);
  systemState.mainState = RESETTING;
  systemState.stateStartTime = millis();
  
  Serial.print("Reset motion started: ");
  Serial.print(reverseDir);
  Serial.print(", ");
  Serial.print(systemState.lastPercent);
  Serial.println("%");
}

void updateMainStateMachine() {
  unsigned long now = millis();
  
  switch (systemState.mainState) {
    case IDLE:
      // 等待命令
      break;
      
    case RUNNING:
      // 检查是否所有组件都完成运动
      bool allDone = true;
      if (systemState.group1.isRunning && (now - systemState.group1.startTime < systemState.group1.duration)) {
        allDone = false;
      }
      if (systemState.group2.isRunning && (now - systemState.group2.startTime < systemState.group2.duration)) {
        allDone = false;
      }
      if (systemState.stepper.isRunning && (now - systemState.stepper.startTime < systemState.stepper.duration)) {
        allDone = false;
      }
      if (systemState.servo.isRunning && (now - systemState.servo.startTime < systemState.servo.duration)) {
        allDone = false;
      }
      
      if (allDone) {
        stopAllMotion();
        systemState.mainState = STOPPING;
        systemState.stateStartTime = now;
        Serial.println("Motion completed, entering stop phase (10s)");
      }
      break;
      
    case STOPPING:
      // 停止10秒
      if (now - systemState.stateStartTime >= 10000) {
        resetMainMotion();
        Serial.println("Stop phase completed, starting reset");
      }
      break;
      
    case RESETTING:
      // 检查复位是否完成
      bool resetDone = true;
      if (systemState.group1.isRunning && (now - systemState.group1.startTime < systemState.group1.duration)) {
        resetDone = false;
      }
      if (systemState.group2.isRunning && (now - systemState.group2.startTime < systemState.group2.duration)) {
        resetDone = false;
      }
      if (systemState.stepper.isRunning && (now - systemState.stepper.startTime < systemState.stepper.duration)) {
        resetDone = false;
      }
      if (systemState.servo.isRunning && (now - systemState.servo.startTime < systemState.servo.duration)) {
        resetDone = false;
      }
      
      if (resetDone) {
        stopAllMotion();
        systemState.mainState = IDLE;
        Serial.println("Reset completed, returning to IDLE");
      }
      break;
  }
}

// === 调试模式控制 ===
void executeDebugCommand(const ParsedCommand &cmd) {
  int speed = 255;  // 调试模式使用100%速度
  unsigned long duration = calculateDuration(cmd.debugPercent);
  unsigned long now = millis();
  
  if (cmd.isGroup1 || cmd.isAll) {
    if (cmd.debugDirection == 'F') {
      moveGroupForward(1, speed);
      systemState.group1.startTime = now;
      systemState.group1.duration = duration;
      systemState.group1.percent = cmd.debugPercent;
    } else if (cmd.debugDirection == 'B') {
      moveGroupBackward(1, speed);
      systemState.group1.startTime = now;
      systemState.group1.duration = duration;
      systemState.group1.percent = cmd.debugPercent;
    } else {
      stopGroup(1);
    }
  }
  
  if (cmd.isGroup2 || cmd.isAll) {
    if (cmd.debugDirection == 'F') {
      moveGroupForward(2, speed);
      systemState.group2.startTime = now;
      systemState.group2.duration = duration;
      systemState.group2.percent = cmd.debugPercent;
    } else if (cmd.debugDirection == 'B') {
      moveGroupBackward(2, speed);
      systemState.group2.startTime = now;
      systemState.group2.duration = duration;
      systemState.group2.percent = cmd.debugPercent;
    } else {
      stopGroup(2);
    }
  }
  
  if (cmd.isStepper || cmd.isAll) {
    if (cmd.debugDirection == 'L') {
      startStepper(true, cmd.debugPercent);
    } else if (cmd.debugDirection == 'R') {
      startStepper(false, cmd.debugPercent);
    } else {
      stopStepper();
    }
  }
  
  if (cmd.isServo || cmd.isAll) {
    if (cmd.debugDirection != 'S') {
      setServoAngle(cmd.servoTargetAngle, cmd.debugPercent);
    }
  }
  
  Serial.print("Debug command executed: ");
  if (cmd.isGroup1) Serial.print("GROUP1,");
  if (cmd.isGroup2) Serial.print("GROUP2,");
  if (cmd.isStepper) Serial.print("STEPPER,");
  if (cmd.isServo) Serial.print("SERVO,");
  if (cmd.isAll) Serial.print("ALL,");
  Serial.print(cmd.debugDirection);
  Serial.print(",");
  Serial.println(cmd.debugPercent);
}

// === 命令解析 ===
bool parseCommand(const String &line, ParsedCommand &out) {
  // 初始化
  out.isMainMode = false;
  out.isStop = false;
  out.isStart = false;
  out.startActuators = false;
  out.startStepper = false;
  out.startServo = false;
  out.startAll = false;
  out.isGroup1 = false;
  out.isGroup2 = false;
  out.isStepper = false;
  out.isServo = false;
  out.isAll = false;
  
  line.trim();
  String upperLine = line;
  upperLine.toUpperCase();
  
  // 强制停止命令 's'
  if (line.length() == 1 && (line[0] == 's' || line[0] == 'S')) {
    out.isStop = true;
    return true;
  }
  
  // START命令：START,ACTUATORS 或 START,STEPPER 或 START,SERVO 或 START,ALL
  if (upperLine.startsWith("START")) {
    out.isStart = true;
    int commaPos = upperLine.indexOf(',');
    if (commaPos > 0) {
      String target = upperLine.substring(commaPos + 1);
      target.trim();
      
      // 检查是否有+号（组合命令）
      int plusPos = target.indexOf('+');
      if (plusPos > 0) {
        // 组合命令：START,ACTUATORS+SERVO
        String part1 = target.substring(0, plusPos);
        String part2 = target.substring(plusPos + 1);
        part1.trim();
        part2.trim();
        
        if (part1 == "ACTUATORS" || part2 == "ACTUATORS") {
          out.startActuators = true;
        }
        if (part1 == "STEPPER" || part2 == "STEPPER") {
          out.startStepper = true;
        }
        if (part1 == "SERVO" || part2 == "SERVO") {
          out.startServo = true;
        }
      } else {
        // 单个目标
        if (target == "ACTUATORS" || target == "ALL") {
          out.startActuators = true;
        }
        if (target == "STEPPER" || target == "ALL") {
          out.startStepper = true;
        }
        if (target == "SERVO" || target == "ALL") {
          out.startServo = true;
        }
        if (target == "ALL") {
          out.startAll = true;
        }
      }
      
      // 至少选择一个目标
      if (out.startActuators || out.startStepper || out.startServo) {
        return true;
      }
    } else {
      // START 单独使用，等同于 START,ALL
      out.startAll = true;
      out.startActuators = true;
      out.startStepper = true;
      out.startServo = true;
      return true;
    }
  }
  
  // 主模式：f, 20 或 b, 20
  int firstComma = line.indexOf(',');
  if (firstComma > 0 && firstComma < 3) {
    String firstPart = line.substring(0, firstComma);
    firstPart.trim();
    firstPart.toUpperCase();
    
    if (firstPart == "F" || firstPart == "B") {
      String percentStr = line.substring(firstComma + 1);
      percentStr.trim();
      int percent = percentStr.toInt();
      percent = constrain(percent, 0, 100);
      
      out.isMainMode = true;
      out.mainDirection = firstPart[0];
      out.mainPercent = percent;
      return true;
    }
  }
  
  // 调试模式：GROUP1,F,20 或 STEPPER,L,30 等
  // 支持组合：GROUP1+STEPPER,F,20
  
  // 检查是否有+号（组合命令）
  int plusPos = upperLine.indexOf('+');
  String targets;
  String rest;
  
  if (plusPos > 0) {
    targets = upperLine.substring(0, plusPos);
    rest = upperLine.substring(plusPos + 1);
    int commaPos = rest.indexOf(',');
    if (commaPos > 0) {
      targets = upperLine.substring(0, plusPos);
      String secondTarget = rest.substring(0, commaPos);
      targets += "+" + secondTarget;
      rest = rest.substring(commaPos);
    }
  } else {
    int firstCommaPos = upperLine.indexOf(',');
    if (firstCommaPos > 0) {
      targets = upperLine.substring(0, firstCommaPos);
      rest = upperLine.substring(firstCommaPos);
    } else {
      return false;
    }
  }
  
  // 解析目标
  if (targets.indexOf("GROUP1") >= 0) out.isGroup1 = true;
  if (targets.indexOf("GROUP2") >= 0) out.isGroup2 = true;
  if (targets.indexOf("STEPPER") >= 0) out.isStepper = true;
  if (targets.indexOf("SERVO") >= 0) out.isServo = true;
  if (targets.indexOf("ALL") >= 0) out.isAll = true;
  
  if (!out.isGroup1 && !out.isGroup2 && !out.isStepper && !out.isServo && !out.isAll) {
    return false;
  }
  
  // 解析方向和百分比
  int comma1 = rest.indexOf(',');
  if (comma1 < 0) return false;
  
  String dirPart = rest.substring(1, comma1);
  dirPart.trim();
  String percentPart = rest.substring(comma1 + 1);
  percentPart.trim();
  
  char dir = dirPart.length() > 0 ? dirPart[0] : 'S';
  int percent = percentPart.length() > 0 ? percentPart.toInt() : 0;
  percent = constrain(percent, 0, 100);
  
  out.debugDirection = dir;
  out.debugPercent = percent;
  
  // 对于舵机，解析角度（格式：SERVO,180,50）
  if (out.isServo) {
    int angleComma = percentPart.indexOf(',');
    if (angleComma > 0) {
      String angleStr = percentPart.substring(0, angleComma);
      String percentStr2 = percentPart.substring(angleComma + 1);
      out.servoTargetAngle = angleStr.toInt();
      out.debugPercent = percentStr2.toInt();
      out.debugPercent = constrain(out.debugPercent, 0, 100);
    } else {
      out.servoTargetAngle = 180;  // 默认180度
    }
  } else {
    out.servoTargetAngle = 180;
  }
  
  return true;
}

void applyCommand(const ParsedCommand &cmd) {
  // 检查控制间隙（除强制停止和START外）
  unsigned long now = millis();
  if (systemState.inCooldown && !cmd.isStop && !cmd.isStart) {
    if (now - systemState.lastCommandTime < 30000) {
      Serial.println("In cooldown period (30s), command ignored");
      return;
    } else {
      systemState.inCooldown = false;
    }
  }
  
  // 强制停止命令
  if (cmd.isStop) {
    stopAllMotion();
    systemState.mainState = IDLE;
    systemState.initState = INIT_IDLE;
    systemState.inCooldown = false;  // 清零控制间隙
    Serial.println("Force stop: all motion stopped, cooldown cleared");
    return;
  }
  
  // START命令
  if (cmd.isStart) {
    bool actuators = cmd.startActuators || cmd.startAll;
    bool stepper = cmd.startStepper || cmd.startAll;
    bool servo = cmd.startServo || cmd.startAll;
    
    resetToInitialState(actuators, stepper, servo);
    systemState.lastCommandTime = now;
    systemState.inCooldown = true;
    return;
  }
  
  // 主模式命令
  if (cmd.isMainMode) {
    if (systemState.mainState != IDLE) {
      Serial.println("System busy, please stop first");
      return;
    }
    startMainMotion(cmd.mainPercent, cmd.mainDirection);
    systemState.lastCommandTime = now;
    systemState.inCooldown = true;
    return;
  }
  
  // 调试模式命令
  executeDebugCommand(cmd);
  systemState.lastCommandTime = now;
  systemState.inCooldown = true;
}

// === 初始状态恢复（非阻塞） ===
void resetToInitialState(bool actuators, bool stepper, bool servo) {
  if (actuators) {
    // 伸缩杆：前进到最底（20cm，约33.3秒）
    moveGroupForward(1, 255);
    moveGroupForward(2, 255);
    systemState.initState = INIT_ACTUATORS_FORWARD;
    systemState.initStartTime = millis();
    systemState.initActuators = true;
    
    systemState.group1.isRunning = true;
    systemState.group1.startTime = systemState.initStartTime;
    systemState.group1.duration = 33300;  // 33.3秒
    systemState.group1.direction = 'F';
    
    systemState.group2.isRunning = true;
    systemState.group2.startTime = systemState.initStartTime;
    systemState.group2.duration = 33300;
    systemState.group2.direction = 'F';
    
    Serial.println("Initializing actuators: moving forward to bottom (33.3s)");
  } else {
    systemState.initActuators = false;
  }
  
  if (stepper) {
    // 步进电机：重置位置计数器为0（不实际移动）
    stepperPosition = 0;
    stopStepper();
    systemState.initStepper = true;
    Serial.println("Stepper position reset to 0");
  } else {
    systemState.initStepper = false;
  }
  
  if (servo) {
    // 舵机：转到0度（立即完成）
    servo.write(0);
    systemState.servo.isRunning = false;
    systemState.initServo = true;
    Serial.println("Servo set to 0 degrees");
  } else {
    systemState.initServo = false;
  }
  
  if (!actuators && !stepper && !servo) {
    systemState.initState = INIT_IDLE;
  }
}

void updateInitStateMachine() {
  if (systemState.initState == INIT_IDLE) {
    return;
  }
  
  unsigned long now = millis();
  
  switch (systemState.initState) {
    case INIT_ACTUATORS_FORWARD:
      // 检查是否完成前进
      if (now - systemState.initStartTime >= 33300) {
        // 开始后退10cm（约16.7秒）
        moveGroupBackward(1, 255);
        moveGroupBackward(2, 255);
        systemState.initState = INIT_ACTUATORS_BACKWARD;
        systemState.initStartTime = now;
        
        systemState.group1.isRunning = true;
        systemState.group1.startTime = now;
        systemState.group1.duration = 16700;  // 16.7秒
        systemState.group1.direction = 'B';
        
        systemState.group2.isRunning = true;
        systemState.group2.startTime = now;
        systemState.group2.duration = 16700;
        systemState.group2.direction = 'B';
        
        Serial.println("Initializing actuators: moving backward to center (16.7s)");
      }
      break;
      
    case INIT_ACTUATORS_BACKWARD:
      // 检查是否完成后退
      if (now - systemState.initStartTime >= 16700) {
        stopGroup(1);
        stopGroup(2);
        systemState.initState = INIT_COMPLETE;
        Serial.println("Actuators initialized to center position");
      }
      break;
      
    case INIT_COMPLETE:
      systemState.initState = INIT_IDLE;
      Serial.println("Initialization complete");
      break;
      
    default:
      break;
  }
}

// === 步进电机位置跟踪更新 ===
void updateStepperPosition() {
  if (stepperRunning) {
    unsigned long nowUs = micros();
    if (nowUs - lastStepperToggleUs >= STEPPER_PULSE_INTERVAL_US) {
      lastStepperToggleUs = nowUs;
      stepperStepLevel = !stepperStepLevel;
      digitalWrite(STEPPER_STEP_PIN, stepperStepLevel);
      
      // 更新位置（每次脉冲翻转计数一次）
      if (stepperStepLevel == HIGH) {
        stepperPosition += (stepperLeft ? -1 : 1);
      }
    }
    
    // 检查是否完成运动
    if (systemState.stepper.isRunning) {
      unsigned long now = millis();
      if (now - systemState.stepper.startTime >= systemState.stepper.duration) {
        stopStepper();
      }
    }
  }
}

// === 主程序 ===
void setup() {
  Serial.begin(115200);
  
  // 初始化所有电机控制引脚
  for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
    pinMode(R_EN_PINS[i], OUTPUT);
    pinMode(L_EN_PINS[i], OUTPUT);
    pinMode(RPWM_PINS[i], OUTPUT);
    pinMode(LPWM_PINS[i], OUTPUT);
    digitalWrite(R_EN_PINS[i], LOW);
    digitalWrite(L_EN_PINS[i], LOW);
    analogWrite(RPWM_PINS[i], 0);
    analogWrite(LPWM_PINS[i], 0);
  }
  
  // 初始化步进电机
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENA_PIN, OUTPUT);
  enableStepper(true);
  stopStepper();
  
  // 初始化舵机
  servo.attach(SERVO_PIN);
  servo.write(0);
  
  // 初始化系统状态
  systemState.mainState = IDLE;
  systemState.initState = INIT_IDLE;
  systemState.inCooldown = false;
  systemState.lastCommandTime = 0;
  systemState.lastPercent = 0;
  
  Serial.println("=== Multi-Component Synchronized Control System ===");
  Serial.println("Main mode: f,<percent> or b,<percent>  (e.g. f,20)");
  Serial.println("Debug mode: GROUP1/GROUP2/STEPPER/SERVO,<dir>,<percent>");
  Serial.println("  Examples: GROUP1,F,20  STEPPER,L,30  SERVO,180,50");
  Serial.println("  Combined: GROUP1+STEPPER,F,20");
  Serial.println("Initial state: START,ACTUATORS/STEPPER/SERVO/ALL");
  Serial.println("  Examples: START,ACTUATORS  START,ALL  START,ACTUATORS+SERVO");
  Serial.println("Force stop: s");
  Serial.println("Ready to receive commands...");
  
  // 上电后不自动执行任何动作，只设置初始值
  servo.write(0);  // 舵机角度设为0（不运动）
  stepperPosition = 0;  // 步进电机位置计数器重置为0（不实际移动）
}

void loop() {
  // 非阻塞读取串口
  while (Serial.available()) {
    char incoming = Serial.read();
    
    if (incoming == '\n' || incoming == '\r') {
      if (cmdIndex == 0) {
        continue;
      }
      
      cmdBuffer[cmdIndex] = '\0';
      String line = String(cmdBuffer);
      cmdIndex = 0;
      
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
        cmdIndex = 0;
      }
    }
  }
  
  // 更新主工作模式状态机
  updateMainStateMachine();
  
  // 更新初始状态恢复状态机
  updateInitStateMachine();
  
  // 更新步进电机位置跟踪
  updateStepperPosition();
  
  // 更新控制间隙状态
  unsigned long now = millis();
  if (systemState.inCooldown && (now - systemState.lastCommandTime >= 30000)) {
    systemState.inCooldown = false;
  }
}
