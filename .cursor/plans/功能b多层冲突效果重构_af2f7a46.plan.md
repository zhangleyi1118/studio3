---
name: 功能B多层冲突效果重构
overview: 重构功能B的虚拟位置系统，实现动态端点拉伸和4层叠加效果（底色/主暗波/湍流/事件），将波相位从1-20扩展到0-21，支持从0开始的连续行波。
todos:
  - id: step1_phase_refactor
    content: 重构波相位系统：将globalVirtualPos改为phase，范围[0,21)，添加n映射和速度控制
    status: completed
  - id: step2_cyclic_distance
    content: 扩展getCyclicDistance函数支持period=21的通用循环距离计算
    status: completed
    dependencies:
      - step1_phase_refactor
  - id: step3_endpoint_stretch
    content: 实现端点动态拉伸：组2(1→0)和组3(20→21)根据n值平滑拉伸
    status: completed
    dependencies:
      - step2_cyclic_distance
  - id: step4_main_darkwave
    content: 实现主暗波层：高斯脉冲下陷效果，波宽和深度随n变化
    status: completed
    dependencies:
      - step3_endpoint_stretch
  - id: step5_overshoot
    content: 添加回弹过冲效果：暗波后沿的亮度回弹
    status: completed
    dependencies:
      - step4_main_darkwave
  - id: step6_turbulence
    content: 实现湍流层：空间相关的伪噪声，随n增大而增强
    status: completed
    dependencies:
      - step5_overshoot
  - id: step7_event_layer
    content: 实现事件层：n>70时偶发的冲击波效果，带cooldown机制
    status: completed
    dependencies:
      - step6_turbulence
  - id: step8_integration
    content: 整合4层效果到renderFunctionB，确保所有层正确叠加
    status: completed
    dependencies:
      - step7_event_layer
---

# 功能B多层冲突效果重构计划

## 目标

将功能B从简单的拖尾效果升级为多层叠加的"红巨星内部冲突"效果，包括：

- 动态端点拉伸（组2: 1→0, 组3: 20→21）
- 波相位从0开始，范围[0,21)
- 4层叠加：底色层、主暗波（高斯脉冲）、湍流、事件层

## 架构变更

### 1. 核心概念分离

- **静态虚拟位置 `pos[i]`**：每个LED的深度坐标（根据n动态拉伸端点）
- **波相位 `phase`**：行波传播坐标，范围[0,21)，从0开始

### 2. 关键参数定义

- 周期：`PHASE_PERIOD = 21.0`
- 速度范围：`PHASE_SPEED_SLOW = 0.22`, `PHASE_SPEED_FAST = 3.2`
- 事件cooldown：`EVENT_COOLDOWN = 1.8` 秒
- 事件频率：n<70时为0，n=70-100时从0.05/s到0.35/s

## 实现步骤

### 步骤1：重构波相位系统

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **修改常量定义**（第57-60行）：

   - 移除 `VIRTUAL_POS_MIN` 和 `VIRTUAL_POS_MAX`
   - 添加 `PHASE_PERIOD = 21.0`
   - 添加 `PHASE_SPEED_SLOW = 0.22`, `PHASE_SPEED_FAST = 3.2`

2. **修改ControlState结构体**（第139-149行）：

   - 将 `globalVirtualPos` 重命名为 `phase`
   - 初始值改为 `0.0`

3. **重构 `updateVirtualPosition()` 函数**（第711-739行）：

   - 使用n映射计算k值：`t = clamp(n/100, 0..1)`, `k = smoothstep(t)` 或 `k = t*t`
   - 速度计算：`speed = lerp(PHASE_SPEED_SLOW, PHASE_SPEED_FAST, k)`
   - 相位更新：`phase = fmod(phase + speed * deltaTime, PHASE_PERIOD)`

### 步骤2：扩展循环距离计算

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **重写 `getCyclicDistance()` 函数**（第238-246行）：
   ```cpp
   float getCyclicDistance(float pos1, float pos2, float period) {
       float dist = fabs(pos1 - pos2);
       return min(dist, period - dist);
   }
   ```


   - 添加period参数，支持任意周期
   - 使用通用算法：`min(d, period - d)`

### 步骤3：实现端点动态拉伸

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **添加端点拉伸计算函数**（在renderFunctionB之前）：
   ```cpp
   float calculateStretchedPos(float basePos, float n) {
       float t = constrain(n / 100.0, 0.0, 1.0);
       float s = smoothstep(t);  // 或 t*t
       
       if (basePos == 1.0) {
           // 组2：1 → 0
           float pos = 1.0 - 1.0 * s;
           return (pos >= 0.0) ? pos : 0.0;
       } else if (basePos == 20.0) {
           // 组3：20 → 21
           float pos = 20.0 + 1.0 * s;
           return (pos < 21.0) ? pos : (21.0 - 1e-4);  // 方案A：限制在[0,21)
       }
       return basePos;  // 其他位置不变
   }
   ```

2. **修改 `renderFunctionB()` 中的位置计算**（第782-792行）：

   - 在计算 `localPos` 后，对端点应用拉伸：
     ```cpp
     if (range.start == range.end && (range.start == 1.0 || range.start == 20.0)) {
         localPos = calculateStretchedPos(localPos, state.currentControlValue);
     }
     ```


### 步骤4：实现主暗波（高斯脉冲下陷）

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **添加主暗波计算函数**：
   ```cpp
   float calculateMainDarkWave(float pos, float phase, float n) {
       float t = constrain(n / 100.0, 0.0, 1.0);
       float k = smoothstep(t);  // 或 t*t
       
       // 计算循环距离
       float d = getCyclicDistance(pos, phase, PHASE_PERIOD);
       
       // 波宽和深度随n变化
       float width = lerpFloat(1.5, 0.8, k);  // n大时更窄
       float depth = lerpFloat(0.3, 0.85, k);   // n大时更深
       
       // 高斯脉冲下陷
       float pulse = exp(-(d * d) / (width * width));
       return depth * pulse;
   }
   ```

2. **在 `renderFunctionB()` 中应用主暗波**：

   - 计算暗波贡献：`darkWave = calculateMainDarkWave(localPos, state.phase, state.currentControlValue)`
   - 亮度调制：`brightness *= (1.0 - darkWave)`

### 步骤5：添加回弹过冲效果

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **扩展主暗波函数，添加回弹**：
   ```cpp
   float calculateOvershoot(float pos, float phase, float n) {
       float t = constrain(n / 100.0, 0.0, 1.0);
       float k = smoothstep(t);
       
       float d = getCyclicDistance(pos, phase, PHASE_PERIOD);
       float offset = 0.4;  // 回弹位置偏移
       float width2 = 0.6;
       float gain = lerpFloat(0.05, 0.25, k);
       
       float overshoot = gain * exp(-((d - offset) * (d - offset)) / (width2 * width2));
       return (d > offset) ? overshoot : 0.0;  // 只在后沿
   }
   ```

2. **在渲染中叠加回弹**：

   - `brightness += calculateOvershoot(...) * baseBrightness`

### 步骤6：实现湍流层（空间相关噪声）

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **添加简单的value-noise函数**：
   ```cpp
   float pseudoNoise(float x) {
       // 简单的哈希函数
       float f = sin(x * 12.9898) * 43758.5453;
       return f - floor(f);  // 返回[0,1)
   }
   
   float smoothNoise(float pos, float time, float n) {
       float t = constrain(n / 100.0, 0.0, 1.0);
       float k = smoothstep(t);
       
       float freq = 0.8;  // 空间频率
       float rate = lerpFloat(0.1, 1.5, k);  // 时间变化率
       float amp = lerpFloat(0.0, 0.15, k);  // 幅度
       
       float noise = pseudoNoise(pos * freq + time * rate);
       return (noise - 0.5) * amp;  // 中心化并缩放
   }
   ```

2. **在渲染中叠加湍流**：

   - 需要添加全局时间变量或使用 `millis() / 1000.0`
   - `brightness += smoothNoise(localPos, globalTime, n) * baseBrightness`

### 步骤7：实现事件层（冲击波/撕裂瞬变）

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

1. **扩展ControlState，添加事件状态**：
   ```cpp
   struct ControlState {
       // ... 现有字段 ...
       float eventIntensity;      // 当前事件强度
       float eventPhase;          // 事件相位（用于扫过效果）
       unsigned long lastEventTime;  // 上次事件时间（用于cooldown）
   };
   ```

2. **添加事件更新函数**：
   ```cpp
   void updateEventLayer(ControlState* state, float deltaTime) {
       float n = state->currentControlValue;
       if (n < 70.0) {
           state->eventIntensity = 0.0;
           return;
       }
       
       // 检查cooldown
       unsigned long now = millis();
       if (state->eventIntensity > 0.0) {
           // 更新现有事件
           state->eventPhase += deltaTime * 8.0;  // 快速扫过
           if (state->eventPhase > PHASE_PERIOD) {
               state->eventIntensity = 0.0;
           }
       } else if (now - state->lastEventTime >= EVENT_COOLDOWN * 1000) {
           // 检查是否触发新事件
           float t = (n - 70.0) / 30.0;
           float rate = lerpFloat(0.05, 0.35, t);  // 每秒概率
           if (randomFloat(0.0, 1.0) < rate * deltaTime) {
               state->eventIntensity = 1.0;
               state->eventPhase = 0.0;
               state->lastEventTime = now;
           }
       }
   }
   ```

3. **添加事件贡献计算函数**：
   ```cpp
   float calculateEventContribution(float pos, ControlState* state) {
       if (state->eventIntensity <= 0.0) return 0.0;
       
       float d = getCyclicDistance(pos, state->eventPhase, PHASE_PERIOD);
       float width = 0.5;  // 窄暗带
       float pulse = exp(-(d * d) / (width * width));
       
       return -state->eventIntensity * pulse * 0.6;  // 压暗
   }
   ```

4. **在loop()中调用事件更新**，在 `renderFunctionB()` 中应用事件贡献

### 步骤8：整合4层效果

**文件**: `灯带/sketch_nov20a/sketch_nov20a.ino`

**重构 `renderFunctionB()` 函数**（第771-804行）：

1. 计算底色层：`baseBrightness = mapControlToBrightnessB(n)`
2. 计算主暗波：`darkWave = calculateMainDarkWave(...)`
3. 计算回弹：`overshoot = calculateOvershoot(...)`
4. 计算湍流：`turbulence = smoothNoise(...)`
5. 计算事件：`event = calculateEventContribution(...)`
6. 最终亮度：`brightness = baseBrightness * (1 - darkWave + overshoot) + turbulence + event`
7. 应用循环距离时使用 `PHASE_PERIOD` 而不是硬编码的20

## 注意事项

1. **性能优化**：

   - 空间相关噪声可以预计算或使用查找表
   - 事件层只在n>70时计算

2. **调试建议**：

   - 先实现步骤1-3，确认波从0开始、端点拉伸正确
   - 再逐步添加各层效果，每层单独测试

3. **参数调优**：

   - 波宽、深度、湍流幅度等参数可能需要根据实际效果微调
   - 建议将这些参数定义为可调常量

4. **兼容性**：

   - 保持功能A不变
   - 保持潮汐桥功能不变