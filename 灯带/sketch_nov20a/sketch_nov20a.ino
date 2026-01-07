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
#define TIDAL_STRIP_1_LEDS    200     // GPIO 22: 2m灯带（200 LEDs）
#define TIDAL_STRIP_2_LEDS    300     // GPIO 23: 3m灯带（300 LEDs）
#define TIDAL_PIN_1           22      // GPIO 22
#define TIDAL_PIN_2           23      // GPIO 23

// 潮汐桥波浪参数
// n=0: 200灯需要2m15s(135秒) = 1.481 LEDs/秒
// n=100: 200灯需要2m5s(125秒) = 1.6 LEDs/秒
#define WAVE_SPEED_MIN         1.48    // n=0时的速度 (LEDs/秒)
#define WAVE_SPEED_MAX         1.6     // n=100时的速度 (LEDs/秒)
#define WAVE_WIDTH              30.0    // 波浪影响宽度（LEDs）
#define RESTART_DELAY_MS        0      // 重置前的停顿时间（毫秒，0表示无停顿）

// 潮汐桥颜色参数
#define RED_RATIO_MIN          0.2     // n=0时红色区域比例（20%）
#define RED_RATIO_MAX          0.5     // n=100时红色区域比例（50%）
#define BLUE_RATIO             0.6     // 寒色区域固定比例（60%）

// 潮汐桥亮度参数
#define BRIGHTNESS_START       30      // 起始位置亮度（0%位置，红色区域）
#define BRIGHTNESS_END         80      // 末端位置亮度（红色和寒色区域结束位置）
#define BRIGHTNESS_WHITE       100     // 白色区域最亮（不算波浪，应该比红色区域亮但不要太亮）
#define WAVE_BOOST_RATIO       0.2     // 波浪亮度提高比例（20%）
#define COLOR_TRANSITION_WIDTH 0.05    // 颜色渐变过渡区域宽度（5%）

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
// 注意：波浪效果现在直接在calculateTidalLEDColor中计算，不再需要查找表

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

// 注意：波浪效果现在直接在calculateTidalLEDColor中计算，不再需要查找表

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
// n=0: 1.48 LEDs/秒, n=100: 1.6 LEDs/秒
float calculateWaveSpeed(float n) {
    return WAVE_SPEED_MIN + (n / 100.0) * (WAVE_SPEED_MAX - WAVE_SPEED_MIN);
}

// 计算红色区域比例
// n=0: 20%, n=100: 50%
float calculateRedRatio(float n) {
    return RED_RATIO_MIN + (n / 100.0) * (RED_RATIO_MAX - RED_RATIO_MIN);
}

// 更新潮汐桥波浪位置
void updateTidalWavePosition(TidalStripState* state, int totalLEDs) {
    // 如果控制值为0，不更新波浪位置
    if (state->controlValue <= 0) {
        return;
    }
    
    // 确保时间基准已初始化
    if (state->lastUpdateTime == 0) {
        state->lastUpdateTime = millis();
        state->wavePosition = 0.0;  // 确保从0开始
        return;
    }
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - state->lastUpdateTime) / 1000.0; // 转换为秒
    
    // 防止deltaTime过大（比如系统重启后）
    if (deltaTime > 1.0) {
        deltaTime = 0.016;  // 限制为约60fps的帧时间
    }
    
    state->lastUpdateTime = currentTime;
    
    // 计算传播速度
    float speed = calculateWaveSpeed(state->controlValue);
    
    // 更新波浪位置
    state->wavePosition += speed * deltaTime;
    
    // 检查是否到达末端（无停顿，立即重新开始）
    while (state->wavePosition >= totalLEDs) {
        state->wavePosition -= totalLEDs;  // 循环回到起始位置
    }
}

// 获取位置在灯带上的颜色（基于长度分布，带渐变过渡）
CHSV getTidalBaseColorByPosition(float position, float n) {
    // position: 0.0-1.0，表示在灯带上的位置
    float redRatio = calculateRedRatio(n);
    float blueRatio = BLUE_RATIO;  // 固定60%是寒色
    float whiteRatio = 1.0 - redRatio - blueRatio;  // 剩余部分是白色
    
    // 计算各区域的边界
    float redEnd = redRatio;
    float blueEnd = redRatio + blueRatio;
    
    // 渐变过渡区域宽度
    float transitionWidth = COLOR_TRANSITION_WIDTH;
    
    if (position < redEnd - transitionWidth) {
        // 红色区域：纯红色
        return CHSV(0, 255, 255);  // HSV: 色相0(红), 饱和度255, 亮度255
    } else if (position < redEnd + transitionWidth) {
        // 红色到蓝色的渐变区域
        float progress = (position - (redEnd - transitionWidth)) / (2.0 * transitionWidth);
        progress = constrain(progress, 0.0, 1.0);
        // 色相从0(红)渐变到160(蓝)
        uint8_t hue = (uint8_t)(0 + progress * 160);
        // 饱和度从255渐变到200
        uint8_t sat = (uint8_t)(255 - progress * 55);
        return CHSV(hue, sat, 255);
    } else if (position < blueEnd - transitionWidth) {
        // 寒色区域：浅蓝色
        return CHSV(160, 200, 255);  // HSV: 色相160(浅蓝), 饱和度200, 亮度255
    } else if (position < blueEnd + transitionWidth) {
        // 蓝色到白色的渐变区域
        float progress = (position - (blueEnd - transitionWidth)) / (2.0 * transitionWidth);
        progress = constrain(progress, 0.0, 1.0);
        // 色相保持160(蓝)，饱和度从200渐变到0(白色)
        uint8_t sat = (uint8_t)(200 - progress * 200);
        return CHSV(160, sat, 255);
    } else {
        // 白色区域：白色
        return CHSV(0, 0, 255);  // HSV: 色相0, 饱和度0(白色), 亮度255
    }
}

// 计算潮汐桥LED的颜色和亮度
CRGB calculateTidalLEDColor(int ledIndex, int totalLEDs, TidalStripState* state) {
    float position = (float)ledIndex / totalLEDs;  // 0.0-1.0
    
    // 1. 获取基础颜色（固定颜色，无波动，不改变）
    CHSV baseColorHSV = getTidalBaseColorByPosition(position, state->controlValue);
    
    // 2. 计算基础亮度（30-80渐变，红色区域保持较低亮度）
    float baseBrightness;
    float redRatio = calculateRedRatio(state->controlValue);
    float blueEnd = redRatio + BLUE_RATIO;
    
    if (position >= blueEnd) {
        // 白色区域：最亮100（不算波浪）
        baseBrightness = BRIGHTNESS_WHITE;
    } else if (position < redRatio) {
        // 红色区域：保持较低亮度30-40（根据位置在红色区域内的比例）
        float redProgress = position / redRatio;  // 在红色区域内的进度0-1
        baseBrightness = BRIGHTNESS_START + redProgress * 10.0;  // 30-40
    } else {
        // 寒色区域：从红色区域结束处(40)渐变到80
        float blueProgress = (position - redRatio) / BLUE_RATIO;  // 在寒色区域内的进度0-1
        baseBrightness = 40.0 + blueProgress * (BRIGHTNESS_END - 40.0);  // 40-80
    }
    
    // 3. 波浪效果：只提高亮度20%，不改变颜色
    // 计算到波浪位置的距离（考虑循环）
    float distToWave = fabs((float)ledIndex - state->wavePosition);
    // 如果距离超过灯带长度的一半，从另一侧计算（考虑循环）
    if (distToWave > totalLEDs / 2.0) {
        distToWave = totalLEDs - distToWave;
    }
    
    float waveBoost = 0.0;
    
    if (distToWave < WAVE_WIDTH) {
        // 波浪影响范围内，使用高斯衰减
        float waveInfluence = exp(-distToWave * distToWave / (WAVE_WIDTH * WAVE_WIDTH * 0.5));
        // 亮度提高20%
        waveBoost = baseBrightness * WAVE_BOOST_RATIO * waveInfluence;
    }
    
    // 4. 整体亮度由n控制（0-100映射到0-1）
    float globalBrightness = state->controlValue / 100.0;
    
    // 5. 最终亮度
    float finalBrightness = (baseBrightness + waveBoost) * globalBrightness;
    finalBrightness = constrain(finalBrightness, 0, 255);
    
    // 6. 保持颜色不变，只改变亮度
    CHSV finalHSV = CHSV(baseColorHSV.h, baseColorHSV.s, (uint8_t)finalBrightness);
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
    // 注意：潮汐桥不再需要查找表，波浪效果直接计算
    
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
            
            // 重置时间基准，确保速度立即改变（避免累积误差）
            state.lastUpdateTime = millis();
            
            // 同时更新潮汐桥的控制值（直接设置，不过渡）
            tidal_strip1_state.controlValue = targetValue;
            tidal_strip2_state.controlValue = targetValue;
            
            // 重置潮汐桥时间基准，确保速度立即改变
            unsigned long now = millis();
            tidal_strip1_state.lastUpdateTime = now;
            tidal_strip2_state.lastUpdateTime = now;
            
            // 如果设置为0，重置波浪位置
            if (targetValue == 0) {
                tidal_strip1_state.wavePosition = 0.0;
                tidal_strip2_state.wavePosition = 0.0;
            }
            
            Serial.print("OK: Set target to ");
            Serial.print(targetValue);
            Serial.print(" (14 strips speed: ");
            Serial.print(targetValue + 10.0);
            Serial.print(" pos/sec, Tidal speed: ");
            Serial.print(calculateWaveSpeed(targetValue), 2);
            Serial.println(" LEDs/sec)");
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
    
    // 防止deltaTime过大（比如系统重启后）
    if (deltaTime > 1.0) {
        deltaTime = 0.016;  // 限制为约60fps的帧时间
    }
    
    state.lastUpdateTime = currentTime;
    
    // 计算速度：v = n + 10
    // 使用targetControlValue确保速度立即改变，而不是平滑过渡
    float speed = state.targetControlValue + 10.0;
    
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
