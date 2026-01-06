/*
 * LED灯带控制系统 - ESP32版本
 * 
 * 分组方案：14根灯带分为5组，同组灯带共用GPIO（硬件并联）
 * - 组1：灯带1-4 → GPIO 4（虚拟位置7-14）
 * - 组2：灯带5 → GPIO 16（虚拟位置1）
 * - 组3：灯带6 → GPIO 17（虚拟位置20）
 * - 组4：灯带7-10 → GPIO 18（虚拟位置15-19）
 * - 组5：灯带11-14 → GPIO 19（虚拟位置2-6）
 * 
 * 潮汐桥控制：使用GPIO 22和GPIO 23
 * - GPIO 22: 2m灯带（120 LEDs，1m=60灯）
 * - GPIO 23: 3m灯带（180 LEDs）
 * 
 * 优势：
 * - 只需5个GPIO，完全在ESP32的8个RMT通道限制内
 * - 同组内灯带显示相同内容（硬件并联）
 * - 代码更简单，性能更好
 * 
 * 接线说明：
 * - 同组内的灯带数据线（DI）并联连接到对应的GPIO
 * - 例如：灯带1、2、3、4的数据线都连接到GPIO 2
 */

#include <FastLED.h>
#include <math.h>

// ================= 配置区域 =================

// 1. 灯带参数
#define NUM_GROUPS      5       // 组数量（同组灯带共用GPIO）
#define NUM_STRIPS      14      // 总灯带数量（用于统计）
#define MAX_LEDS_PER_STRIP  500  // 每根灯带的最大灯珠数（用于数组声明）
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define MAX_BRIGHTNESS  255     // FastLED最大亮度值

// 2. 每组的实际灯珠数量（请根据实际情况修改）
// 组索引0-4对应：组1(灯带1-4), 组2(灯带5), 组3(灯带6), 组4(灯带7-10), 组5(灯带11-14)
// 同组内所有灯带长度相同，共用同一个数据数组
const int LEDS_PER_GROUP[NUM_GROUPS] = {
    300,  // 组1：灯带1-4（请根据实际情况修改）
    300,  // 组2：灯带5（请根据实际情况修改）
    300,  // 组3：灯带6（请根据实际情况修改）
    300,  // 组4：灯带7-10（请根据实际情况修改）
    300   // 组5：灯带11-14（请根据实际情况修改）
};

// 3. 分段比例（所有灯带统一）
// a = 0%, b = 25%, c = 75%, d = 100%
// 分段位置会根据每根灯带的实际灯珠数动态计算

// 3. 虚拟位置参数
#define VIRTUAL_POS_MIN  1.0
#define VIRTUAL_POS_MAX  20.0
#define TRAIL_RANGE      2.0    // 拖尾前后各2个虚拟位置

// 4. 亮度过渡参数
#define TRANSITION_TIME_MS  16000  // 0→100需要16秒
#define TRANSITION_UPDATE_INTERVAL  20  // 每20ms更新一次

// 5. 潮汐桥参数
#define TIDAL_STRIP_1_LEDS    120     // GPIO 22: 2m灯带（120 LEDs）
#define TIDAL_STRIP_2_LEDS    180     // GPIO 23: 3m灯带（180 LEDs）
#define TIDAL_PIN_1           22      // GPIO 22
#define TIDAL_PIN_2           23      // GPIO 23

// 潮汐桥波浪参数
#define WAVE_SPEED_BASE         10.0    // 基础传播速度 (LEDs/秒)
#define WAVE_SPEED_MULTIPLIER   0.5     // 速度随n的增量
#define WAVE_WIDTH              30.0    // 波浪影响宽度（LEDs）
#define WAVE_DECAY              0.3     // 拖尾衰减系数
#define RESTART_DELAY_MS        500     // 重置前的停顿时间（毫秒）

// 潮汐桥颜色参数
#define RED_RATIO_BASE          0.2     // 基础红色区域比例（20%）
#define RED_RATIO_INCREMENT     0.003   // 红色比例随n的增量
#define COLOR_VARIATION         5.0     // 颜色波动范围（度）

// 潮汐桥亮度参数
#define BRIGHTNESS_START_RATIO  0.3     // 起始位置亮度比例
#define BRIGHTNESS_END_RATIO    1.0     // 末端位置亮度比例

// 6. 定义5个组的GPIO引脚（同组灯带硬件并联到同一GPIO）
const uint8_t GROUP_PINS[NUM_GROUPS] = {
    4,   // 组1：灯带1-4 → GPIO 4
    16,  // 组2：灯带5 → GPIO 16
    17,  // 组3：灯带6 → GPIO 17
    18,  // 组4：灯带7-10 → GPIO 18
    19   // 组5：灯带11-14 → GPIO 19
};

// 7. 组到虚拟位置的映射
// 组索引0-4对应：组1(1-4), 组2(5), 组3(6), 组4(7-10), 组5(11-14)
struct VirtualPosRange {
    float start;  // 起始虚拟位置
    float end;    // 结束虚拟位置
};

const VirtualPosRange GROUP_VIRTUAL_MAP[NUM_GROUPS] = {
    {7.0, 14.0},   // 组1：灯带1-4 → 虚拟位置7-14
    {1.0, 1.0},    // 组2：灯带5 → 虚拟位置1
    {20.0, 20.0},  // 组3：灯带6 → 虚拟位置20
    {15.0, 19.0},  // 组4：灯带7-10 → 虚拟位置15-19
    {2.0, 6.0}     // 组5：灯带11-14 → 虚拟位置2-6
};

// ===========================================

// 灯珠数据二维数组：[第几组][第几颗灯]
// 同组内的所有灯带共享相同的数据（硬件并联）
CRGB leds[NUM_GROUPS][MAX_LEDS_PER_STRIP];

// 潮汐桥灯带数据数组
CRGB leds_tidal1[TIDAL_STRIP_1_LEDS];
CRGB leds_tidal2[TIDAL_STRIP_2_LEDS];

// 全局控制状态
struct ControlState {
    float targetControlValue;    // 目标控制值 (0-100)
    float currentControlValue;    // 当前控制值 (0-100)
    unsigned long transitionStartTime;  // 过渡开始时间
    float transitionStartValue;  // 过渡开始时的值
    bool isTransitioning;        // 是否正在过渡
    
    float globalVirtualPos;      // 全局虚拟位置 (1.0-20.0)
    bool isPaused;               // 是否暂停
    unsigned long lastUpdateTime; // 上次更新时间（用于计算deltaTime）
};

ControlState state = {
    .targetControlValue = 0.0,
    .currentControlValue = 0.0,
    .transitionStartTime = 0,
    .transitionStartValue = 0.0,
    .isTransitioning = false,
    .globalVirtualPos = 1.0,
    .isPaused = false,
    .lastUpdateTime = 0
};

// 潮汐桥控制状态
struct TidalStripState {
    float wavePosition;          // 当前波浪位置（LED索引，浮点数）
    float controlValue;          // 当前控制值n (0-100)
    unsigned long lastUpdateTime; // 上次更新时间
    bool isRestarting;          // 是否正在重置（停顿中）
    unsigned long restartStartTime; // 重置开始时间
};

TidalStripState tidal_strip1_state = {
    .wavePosition = 0.0,
    .controlValue = 50.0,  // 默认值50，自主运行
    .lastUpdateTime = 0,
    .isRestarting = false,
    .restartStartTime = 0
};

TidalStripState tidal_strip2_state = {
    .wavePosition = 0.0,
    .controlValue = 50.0,  // 默认值50，自主运行
    .lastUpdateTime = 0,
    .isRestarting = false,
    .restartStartTime = 0
};

// 注意：ESP32的GPIO22就是数字22，不是D22（D22是Arduino Mega的命名方式）

// 潮汐桥拖尾亮度查找表
float tidalTrailBrightnessTable[61];  // 距离0-60，步长1

// 拖尾衰减查找表（预计算，避免实时计算指数函数）
// 距离0-2.0，步长0.1，共21个值
float trailBrightnessTable[21];

void setupTrailTable() {
    // 使用指数衰减：brightness = exp(-distance^2 / decay_factor)
    // decay_factor 控制衰减速度，值越小衰减越快
    float decayFactor = 0.5;
    for (int i = 0; i <= 20; i++) {
        float distance = i * 0.1;
        float brightness = exp(-distance * distance / decayFactor);
        trailBrightnessTable[i] = brightness;
    }
}

// 初始化潮汐桥拖尾查找表
void setupTidalTrailTable() {
    for (int i = 0; i <= 60; i++) {
        float distance = (float)i;
        float brightness = exp(-distance * distance / (WAVE_DECAY * WAVE_WIDTH * WAVE_WIDTH));
        tidalTrailBrightnessTable[i] = brightness;
    }
}

// 根据距离获取潮汐桥拖尾亮度（0.0-1.0）
float getTidalTrailBrightness(float distance) {
    if (distance >= WAVE_WIDTH) return 0.0;
    
    int index = (int)distance;
    if (index > 60) index = 60;
    
    return tidalTrailBrightnessTable[index];
}

// 根据距离获取拖尾亮度（0.0-1.0）
float getTrailBrightness(float distance) {
    if (distance >= TRAIL_RANGE) return 0.0;
    
    int index = (int)(distance * 10.0);
    if (index > 20) index = 20;
    
    return trailBrightnessTable[index];
}

// 计算循环距离（考虑1和20相邻）
float getCyclicDistance(float pos1, float pos2) {
    float dist = fabs(pos1 - pos2);
    // 如果距离大于10，说明应该从另一侧绕
    if (dist > 10.0) {
        dist = VIRTUAL_POS_MAX - dist;
    }
    return dist;
}

// 将控制值映射到功能A的亮度 (0-100 → 20-40)
uint8_t mapControlToBrightnessA(float controlValue) {
    // 控制值0-100映射到20-40
    float brightness = 20.0 + (controlValue / 100.0) * 20.0;
    return (uint8_t)constrain(brightness, 20, 40);
}

// 将控制值映射到功能B的亮度 (0-100 → 0-100)
uint8_t mapControlToBrightnessB(float controlValue) {
    // 控制值0-100直接映射到0-100
    return (uint8_t)constrain(controlValue, 0, 100);
}

// ================= 潮汐桥函数 =================

// 计算波浪传播速度
float calculateWaveSpeed(float n) {
    return WAVE_SPEED_BASE + n * WAVE_SPEED_MULTIPLIER;
}

// 计算红色区域比例
float calculateRedRatio(float n) {
    float ratio = RED_RATIO_BASE + n * RED_RATIO_INCREMENT;
    return constrain(ratio, 0.0, 0.5);  // 限制在0-50%之间
}

// 更新潮汐桥波浪位置
void updateTidalWavePosition(TidalStripState* state, int totalLEDs) {
    // 如果控制值为0，不更新波浪位置
    if (state->controlValue <= 0) {
        return;
    }
    
    if (state->isRestarting) {
        // 检查是否结束停顿
        if (millis() - state->restartStartTime >= RESTART_DELAY_MS) {
            state->isRestarting = false;
            state->wavePosition = 0.0;
            state->lastUpdateTime = millis();
        }
        return;
    }
    
    // 确保时间基准已初始化
    if (state->lastUpdateTime == 0) {
        state->lastUpdateTime = millis();
        return;
    }
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - state->lastUpdateTime) / 1000.0; // 转换为秒
    state->lastUpdateTime = currentTime;
    
    // 计算传播速度
    float speed = calculateWaveSpeed(state->controlValue);
    
    // 更新波浪位置
    state->wavePosition += speed * deltaTime;
    
    // 检查是否到达末端
    if (state->wavePosition >= totalLEDs) {
        state->isRestarting = true;
        state->restartStartTime = millis();
        state->wavePosition = totalLEDs;  // 保持在末端位置
    }
}

// 获取位置在灯带上的颜色（基于长度分布）
CHSV getTidalBaseColorByPosition(float position, float n) {
    // position: 0.0-1.0，表示在灯带上的位置
    float redRatio = calculateRedRatio(n);
    float blueRatio = 0.6;  // 中间60%是浅蓝色
    float whiteRatio = 1.0 - redRatio - blueRatio;  // 剩余部分是白色
    
    // 计算各区域的边界
    float redEnd = redRatio;
    float blueEnd = redRatio + blueRatio;
    
    if (position < redEnd) {
        // 红色区域
        float progress = position / redEnd;  // 0.0-1.0
        // 添加轻微的颜色波动
        float hueVariation = sin(millis() / 1000.0 + position * 10.0) * COLOR_VARIATION;
        return CHSV(0 + hueVariation, 255, 255);  // 红色，HSV模式
    } else if (position < blueEnd) {
        // 浅蓝色区域
        float progress = (position - redEnd) / blueRatio;  // 0.0-1.0
        // 从红色过渡到浅蓝色
        float hue = 0 + progress * 180;  // 0°(红) → 180°(青/浅蓝)
        float hueVariation = sin(millis() / 800.0 + position * 8.0) * COLOR_VARIATION;
        return CHSV(hue + hueVariation, 200, 255);  // 浅蓝色，降低饱和度
    } else {
        // 白色区域
        float progress = (position - blueEnd) / whiteRatio;  // 0.0-1.0
        // 从浅蓝色过渡到白色（降低饱和度）
        float saturation = 200 - progress * 200;  // 200 → 0
        float hueVariation = sin(millis() / 1200.0 + position * 6.0) * COLOR_VARIATION;
        return CHSV(180 + hueVariation, saturation, 255);
    }
}

// 计算潮汐桥LED的颜色和亮度
CRGB calculateTidalLEDColor(int ledIndex, int totalLEDs, TidalStripState* state) {
    float position = (float)ledIndex / totalLEDs;  // 0.0-1.0
    
    // 1. 获取基础颜色（基于长度分布）
    CHSV baseColorHSV = getTidalBaseColorByPosition(position, state->controlValue);
    
    // 2. 计算波浪影响
    float distToWave = fabs((float)ledIndex - state->wavePosition);
    float waveInfluence = getTidalTrailBrightness(distToWave);
    
    // 3. 波前颜色（红色，更鲜艳）
    CHSV waveColorHSV = CHSV(0, 255, 255);  // 纯红色
    
    // 4. 混合颜色（基于波浪影响）
    float waveHue = waveColorHSV.h;
    float baseHue = baseColorHSV.h;
    float mixedHue;
    
    // 色相插值（考虑色相是循环的）
    float hueDiff = waveHue - baseHue;
    if (hueDiff > 128) hueDiff -= 256;
    if (hueDiff < -128) hueDiff += 256;
    mixedHue = baseHue + hueDiff * waveInfluence;
    if (mixedHue < 0) mixedHue += 256;
    if (mixedHue >= 256) mixedHue -= 256;
    
    float mixedSat = baseColorHSV.s + (waveColorHSV.s - baseColorHSV.s) * waveInfluence;
    mixedSat = constrain(mixedSat, 0, 255);
    
    // 5. 计算亮度
    // 静态亮度分布（从头到尾变亮）
    float staticBrightness = BRIGHTNESS_START_RATIO + 
                            (position * (BRIGHTNESS_END_RATIO - BRIGHTNESS_START_RATIO));
    
    // 波浪亮度（波前增强，拖尾衰减但不完全消失）
    float waveBrightness = 0.3 + waveInfluence * 0.7;  // 0.3-1.0范围
    
    // 整体亮度（由n控制）
    float baseBrightness = state->controlValue / 100.0;
    
    // 最终亮度
    float finalBrightness = staticBrightness * waveBrightness * baseBrightness;
    finalBrightness = constrain(finalBrightness, 0.0, 1.0);
    
    uint8_t brightnessValue = (uint8_t)(finalBrightness * 255);
    
    // 6. 转换为RGB
    CHSV finalHSV = CHSV((uint8_t)mixedHue, (uint8_t)mixedSat, brightnessValue);
    CRGB finalRGB;
    hsv2rgb_spectrum(finalHSV, finalRGB);
    
    return finalRGB;
}

// 渲染潮汐桥灯带
void renderTidalStrip(CRGB* leds, int totalLEDs, TidalStripState* state) {
    // 更新波浪位置
    updateTidalWavePosition(state, totalLEDs);
    
    // 计算每个LED的颜色
    for (int i = 0; i < totalLEDs; i++) {
        leds[i] = calculateTidalLEDColor(i, totalLEDs, state);
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("LED Strip Control System Started");
    Serial.print("Groups: "); Serial.println(NUM_GROUPS);
    Serial.print("Total Strips: "); Serial.println(NUM_STRIPS);
    
    // 初始化拖尾查找表
    setupTrailTable();
    setupTidalTrailTable();
    
    // 初始化FastLED（5组，每组一个GPIO）
    // 同组内的灯带通过硬件并联连接到同一个GPIO
    // 使用GPIO引脚：4, 16, 17, 18, 19
    FastLED.addLeds<WS2812B, 4,  COLOR_ORDER>(leds[0], LEDS_PER_GROUP[0]);  // 组1：灯带1-4
    FastLED.addLeds<WS2812B, 16, COLOR_ORDER>(leds[1], LEDS_PER_GROUP[1]);  // 组2：灯带5
    FastLED.addLeds<WS2812B, 17, COLOR_ORDER>(leds[2], LEDS_PER_GROUP[2]);  // 组3：灯带6
    FastLED.addLeds<WS2812B, 18, COLOR_ORDER>(leds[3], LEDS_PER_GROUP[3]);  // 组4：灯带7-10
    FastLED.addLeds<WS2812B, 19, COLOR_ORDER>(leds[4], LEDS_PER_GROUP[4]);  // 组5：灯带11-14
    
    // 初始化潮汐桥灯带（GPIO 22和23）
    // 注意：ESP32的GPIO22就是数字22，不是D22（D22是Arduino Mega的命名方式）
    FastLED.addLeds<WS2812B, TIDAL_PIN_1, COLOR_ORDER>(leds_tidal1, TIDAL_STRIP_1_LEDS);
    FastLED.addLeds<WS2812B, TIDAL_PIN_2, COLOR_ORDER>(leds_tidal2, TIDAL_STRIP_2_LEDS);
    
    Serial.print("Tidal Bridge: GPIO ");
    Serial.print(TIDAL_PIN_1);
    Serial.print(" (");
    Serial.print(TIDAL_STRIP_1_LEDS);
    Serial.print(" LEDs), GPIO ");
    Serial.print(TIDAL_PIN_2);
    Serial.print(" (");
    Serial.print(TIDAL_STRIP_2_LEDS);
    Serial.println(" LEDs)");
    
    FastLED.setBrightness(MAX_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    
    state.lastUpdateTime = millis();
    
    // 初始化潮汐桥时间基准（自主运行，默认值50）
    unsigned long now = millis();
    tidal_strip1_state.lastUpdateTime = now;
    tidal_strip2_state.lastUpdateTime = now;
    
    // 初始化潮汐桥波浪位置（从0开始）
    tidal_strip1_state.wavePosition = 0.0;
    tidal_strip2_state.wavePosition = 0.0;
    
    Serial.println("Tidal Bridge: Auto-running with default n=50");
    
    Serial.println("Initialization complete. Ready for commands.");
    Serial.println("Commands: f,<0-100> | s | q");
    Serial.println("  f,n: 控制14根灯带（功能A和B）+ 潮汐桥（GPIO 22和23，自主运行）");
}

// 解析串口命令
void parseCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd.length() == 0) return;
    
    if (cmd.startsWith("f,")) {
        // f,n 命令：设置目标亮度（同时控制14根灯带和潮汐桥）
        int commaPos = cmd.indexOf(',');
        if (commaPos >= 0) {
            String valueStr = cmd.substring(commaPos + 1);
            float targetValue = valueStr.toFloat();
            targetValue = constrain(targetValue, 0.0, 100.0);
            
            // 更新14根灯带的状态（原有功能）
            state.transitionStartValue = state.currentControlValue;
            state.targetControlValue = targetValue;
            state.transitionStartTime = millis();
            state.isTransitioning = true;
            
            // 同时更新潮汐桥的控制值（直接设置，不过渡）
            tidal_strip1_state.controlValue = targetValue;
            tidal_strip2_state.controlValue = targetValue;
            
            // 如果设置非零值，确保潮汐桥时间基准已初始化
            if (targetValue > 0) {
                unsigned long now = millis();
                if (tidal_strip1_state.lastUpdateTime == 0) {
                    tidal_strip1_state.lastUpdateTime = now;
                }
                if (tidal_strip2_state.lastUpdateTime == 0) {
                    tidal_strip2_state.lastUpdateTime = now;
                }
            }
            
            Serial.print("OK: Set target to ");
            Serial.print(targetValue);
            Serial.println(" (14 strips + tidal bridge)");
        }
    }
    else if (cmd == "s") {
        // s 命令：暂停/恢复
        state.isPaused = !state.isPaused;
        if (state.isPaused) {
            Serial.println("OK: Paused");
        } else {
            Serial.println("OK: Resumed");
            state.lastUpdateTime = millis(); // 重置时间基准
        }
    }
    else if (cmd == "q") {
        // q 命令：退出（关闭所有灯）
        state.targetControlValue = 0.0;
        state.currentControlValue = 0.0;
        state.isTransitioning = false;
        tidal_strip1_state.controlValue = 0.0;
        tidal_strip2_state.controlValue = 0.0;
        FastLED.clear();
        FastLED.show();
        Serial.println("OK: Quit - All LEDs off");
    }
    else {
        Serial.print("ERROR: Unknown command: ");
        Serial.println(cmd);
    }
}

// 更新亮度过渡
void updateBrightnessTransition() {
    if (!state.isTransitioning) return;
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - state.transitionStartTime;
    
    if (elapsed >= TRANSITION_TIME_MS) {
        // 过渡完成
        state.currentControlValue = state.targetControlValue;
        state.isTransitioning = false;
    } else {
        // 线性插值
        float progress = (float)elapsed / TRANSITION_TIME_MS;
        state.currentControlValue = state.transitionStartValue + 
                                   (state.targetControlValue - state.transitionStartValue) * progress;
    }
}

// 更新虚拟位置
void updateVirtualPosition() {
    if (state.isPaused) return;
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - state.lastUpdateTime) / 1000.0; // 转换为秒
    state.lastUpdateTime = currentTime;
    
    // 计算速度：v = n + 10
    float speed = state.currentControlValue + 10.0;
    
    // 更新虚拟位置
    state.globalVirtualPos += speed * deltaTime;
    
    // 循环处理：1.0-20.0
    while (state.globalVirtualPos > VIRTUAL_POS_MAX) {
        state.globalVirtualPos -= (VIRTUAL_POS_MAX - VIRTUAL_POS_MIN + 1.0);
    }
    while (state.globalVirtualPos < VIRTUAL_POS_MIN) {
        state.globalVirtualPos += (VIRTUAL_POS_MAX - VIRTUAL_POS_MIN + 1.0);
    }
}

// 渲染功能A（间隔点亮）
void renderFunctionA(int groupIndex) {
    uint8_t brightnessA = mapControlToBrightnessA(state.currentControlValue);
    int totalLeds = LEDS_PER_GROUP[groupIndex];
    
    // 动态计算分段位置（a=0%, b=25%, c=75%, d=100%）
    int segAStart = 0;                              // a = 0%
    int segBStart = (int)(totalLeds * 0.25);        // b = 25%
    int segCStart = (int)(totalLeds * 0.75);        // c = 75%
    int segDEnd = totalLeds;                       // d = 100%
    
    // 区域1：a→b (0% - 25%)
    for (int i = segAStart; i < segBStart; i++) {
        if (i % 3 == 0) {  // 每隔2颗亮1颗（每3颗中第1颗）
            leds[groupIndex][i] = CHSV(0, 255, brightnessA); // 红色，HSV模式（色相0=红色）
        } else {
            leds[groupIndex][i] = CRGB::Black;
        }
    }
    
    // 区域2：c→d (75% - 100%)
    for (int i = segCStart; i < segDEnd; i++) {
        if (i % 3 == 0) {  // 每隔2颗亮1颗
            leds[groupIndex][i] = CHSV(0, 255, brightnessA); // 红色
        } else {
            leds[groupIndex][i] = CRGB::Black;
        }
    }
}

// 渲染功能B（虚拟位置系统）
void renderFunctionB(int groupIndex) {
    uint8_t baseBrightnessB = mapControlToBrightnessB(state.currentControlValue);
    VirtualPosRange range = GROUP_VIRTUAL_MAP[groupIndex];
    int totalLeds = LEDS_PER_GROUP[groupIndex];
    
    // 动态计算分段位置（b=25%, c=75%）
    int bStart = (int)(totalLeds * 0.25);  // b = 25%
    int cEnd = (int)(totalLeds * 0.75);    // c = 75%
    int bSegmentLength = cEnd - bStart;
    
    for (int i = bStart; i < cEnd; i++) {
        // 将灯珠索引映射到虚拟位置
        float localPos;
        if (range.start == range.end) {
            // 单个虚拟位置（如组2显示位置1）
            localPos = range.start;
        } else {
            // 多个虚拟位置范围（线性映射）
            float progress = (float)(i - bStart) / bSegmentLength;
            localPos = range.start + (range.end - range.start) * progress;
        }
        
        // 计算到全局虚拟位置的距离（考虑循环）
        float distance = getCyclicDistance(localPos, state.globalVirtualPos);
        
        // 获取拖尾亮度
        float trailBrightness = getTrailBrightness(distance);
        
        // 应用亮度
        uint8_t finalBrightness = (uint8_t)(baseBrightnessB * trailBrightness);
        leds[groupIndex][i] = CHSV(0, 255, finalBrightness); // 红色，HSV模式（色相0=红色）
    }
}

void loop() {
    // 1. 处理串口命令
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        parseCommand(cmd);
    }
    
    // 2. 更新亮度过渡
    EVERY_N_MILLISECONDS(TRANSITION_UPDATE_INTERVAL) {
        updateBrightnessTransition();
    }
    
    // 3. 更新虚拟位置
    updateVirtualPosition();
    
    // 4. 渲染所有组（5组）
    for (int g = 0; g < NUM_GROUPS; g++) {
        int totalLeds = LEDS_PER_GROUP[g];
        // 先清空
        for (int i = 0; i < totalLeds; i++) {
            leds[g][i] = CRGB::Black;
        }
        
        // 渲染功能A
        renderFunctionA(g);
        
        // 渲染功能B
        renderFunctionB(g);
    }
    
    // 5. 渲染潮汐桥（GPIO 22和23）
    // 注意：即使controlValue为0，也更新波浪位置，这样一旦设置值后能立即看到效果
    // 但只有当controlValue > 0时才显示灯光
    updateTidalWavePosition(&tidal_strip1_state, TIDAL_STRIP_1_LEDS);
    updateTidalWavePosition(&tidal_strip2_state, TIDAL_STRIP_2_LEDS);
    
    if (tidal_strip1_state.controlValue > 0) {
        // 计算每个LED的颜色
        for (int i = 0; i < TIDAL_STRIP_1_LEDS; i++) {
            leds_tidal1[i] = calculateTidalLEDColor(i, TIDAL_STRIP_1_LEDS, &tidal_strip1_state);
        }
    } else {
        // 如果控制值为0，清空灯带
        for (int i = 0; i < TIDAL_STRIP_1_LEDS; i++) {
            leds_tidal1[i] = CRGB::Black;
        }
    }
    
    if (tidal_strip2_state.controlValue > 0) {
        // 计算每个LED的颜色
        for (int i = 0; i < TIDAL_STRIP_2_LEDS; i++) {
            leds_tidal2[i] = calculateTidalLEDColor(i, TIDAL_STRIP_2_LEDS, &tidal_strip2_state);
        }
    } else {
        // 如果控制值为0，清空灯带
        for (int i = 0; i < TIDAL_STRIP_2_LEDS; i++) {
            leds_tidal2[i] = CRGB::Black;
        }
    }
    
    // 6. 更新显示
    FastLED.show();
}
