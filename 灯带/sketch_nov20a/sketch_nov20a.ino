/*
 * LED灯带控制系统 - ESP32版本
 * 
 * ⚠️ 重要提示：ESP32 RMT通道限制
 * ESP32只有8个RMT通道，但本系统需要控制14根灯带。
 * 
 * 解决方案：
 * 1. 确保使用FastLED库版本 3.5.0 或更高
 * 2. FastLED 3.5+会自动使用I2S驱动来支持超过8个通道
 * 3. 如果遇到"no free tx channels"错误，请升级FastLED库：
 *    - Arduino IDE: 工具 → 管理库 → 搜索"FastLED" → 更新到最新版本
 *    - 或手动下载：https://github.com/FastLED/FastLED/releases
 * 
 * 如果FastLED版本不支持I2S，可以：
 * - 方案A：升级FastLED库（推荐）
 * - 方案B：减少灯带数量到8根或更少
 * - 方案C：使用多个ESP32分别控制部分灯带
 */

#include <FastLED.h>
#include <math.h>

// ================= 配置区域 =================

// 1. 灯带参数
#define NUM_STRIPS      14      // 接口数量
#define MAX_LEDS_PER_STRIP  500  // 每根灯带的最大灯珠数（用于数组声明）
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define MAX_BRIGHTNESS  255     // FastLED最大亮度值

// 2. 每根灯带的实际灯珠数量（请根据实际情况修改）
// 数组索引0-13对应灯带1-14
// 例如：如果灯带1有200颗，灯带2有350颗，则修改为：
// {200, 350, 300, 300, ...}
const int LEDS_PER_STRIP[NUM_STRIPS] = {
    300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300, 300
    // ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑    ↑
    // 灯带1 灯带2 灯带3 灯带4 灯带5 灯带6 灯带7 灯带8 灯带9 灯带10 灯带11 灯带12 灯带13 灯带14
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

// 4. 定义14个接口的引脚（从小到大：灯带1对应最小，灯带14对应最大）
const uint8_t STRIP_PINS[NUM_STRIPS] = {
    2,   // 灯带1  → GPIO 2  (最小)
    4,   // 灯带2  → GPIO 4
    5,   // 灯带3  → GPIO 5
    12,  // 灯带4  → GPIO 12
    13,  // 灯带5  → GPIO 13
    14,  // 灯带6  → GPIO 14
    15,  // 灯带7  → GPIO 15
    16,  // 灯带8  → GPIO 16
    17,  // 灯带9  → GPIO 17
    25,  // 灯带10 → GPIO 25
    26,  // 灯带11 → GPIO 26
    27,  // 灯带12 → GPIO 27
    32,  // 灯带13 → GPIO 32
    33   // 灯带14 → GPIO 33 (最大)
};

// 5. 灯带到虚拟位置的映射
// 灯带索引0-13对应灯带1-14
struct VirtualPosRange {
    float start;  // 起始虚拟位置
    float end;    // 结束虚拟位置
};

const VirtualPosRange STRIP_VIRTUAL_MAP[NUM_STRIPS] = {
    {7.0, 14.0},   // 灯带1 (索引0) → 虚拟位置7-14
    {7.0, 14.0},   // 灯带2 (索引1) → 虚拟位置7-14
    {7.0, 14.0},   // 灯带3 (索引2) → 虚拟位置7-14
    {7.0, 14.0},   // 灯带4 (索引3) → 虚拟位置7-14
    {1.0, 1.0},    // 灯带5 (索引4) → 虚拟位置1
    {20.0, 20.0},  // 灯带6 (索引5) → 虚拟位置20
    {15.0, 19.0},  // 灯带7 (索引6) → 虚拟位置15-19
    {15.0, 19.0},  // 灯带8 (索引7) → 虚拟位置15-19
    {15.0, 19.0},  // 灯带9 (索引8) → 虚拟位置15-19
    {15.0, 19.0},  // 灯带10 (索引9) → 虚拟位置15-19
    {2.0, 6.0},    // 灯带11 (索引10) → 虚拟位置2-6
    {2.0, 6.0},    // 灯带12 (索引11) → 虚拟位置2-6
    {2.0, 6.0},    // 灯带13 (索引12) → 虚拟位置2-6
    {2.0, 6.0}     // 灯带14 (索引13) → 虚拟位置2-6
};

// ===========================================

// 灯珠数据二维数组：[第几条带][第几颗灯]
CRGB leds[NUM_STRIPS][MAX_LEDS_PER_STRIP];

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

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("LED Strip Control System Started");
    
    // 初始化拖尾查找表
    setupTrailTable();
    
    // 初始化FastLED
    // ⚠️ 重要：ESP32只有8个RMT通道，但我们需要14根灯带
    // 解决方案：使用FastLED的I2S驱动（需要FastLED 3.5+版本）
    // 如果FastLED版本较旧，请升级到最新版本
    
    // 方法：使用WS2812Controller800Khz，FastLED会自动选择RMT或I2S
    // 前8根使用RMT，超过8根后自动使用I2S（如果FastLED版本支持）
    
    // 如果遇到"no free tx channels"错误，说明FastLED版本不支持I2S
    // 解决方案：
    // 1. 升级FastLED库到3.5.0或更高版本
    // 2. 或者减少灯带数量到8根或更少
    
    FastLED.addLeds<WS2812B, 2,  COLOR_ORDER>(leds[0], LEDS_PER_STRIP[0]);
    FastLED.addLeds<WS2812B, 4,  COLOR_ORDER>(leds[1], LEDS_PER_STRIP[1]);
    FastLED.addLeds<WS2812B, 5,  COLOR_ORDER>(leds[2], LEDS_PER_STRIP[2]);
    FastLED.addLeds<WS2812B, 12, COLOR_ORDER>(leds[3], LEDS_PER_STRIP[3]);
    FastLED.addLeds<WS2812B, 13, COLOR_ORDER>(leds[4], LEDS_PER_STRIP[4]);
    FastLED.addLeds<WS2812B, 14, COLOR_ORDER>(leds[5], LEDS_PER_STRIP[5]);
    FastLED.addLeds<WS2812B, 15, COLOR_ORDER>(leds[6], LEDS_PER_STRIP[6]);
    FastLED.addLeds<WS2812B, 16, COLOR_ORDER>(leds[7], LEDS_PER_STRIP[7]);
    
    // 第9-14根：如果FastLED支持I2S，这些会使用I2S驱动
    // 如果不支持，这里会报错，需要升级FastLED库
    FastLED.addLeds<WS2812B, 17, COLOR_ORDER>(leds[8], LEDS_PER_STRIP[8]);
    FastLED.addLeds<WS2812B, 25, COLOR_ORDER>(leds[9], LEDS_PER_STRIP[9]);
    FastLED.addLeds<WS2812B, 26, COLOR_ORDER>(leds[10], LEDS_PER_STRIP[10]);
    FastLED.addLeds<WS2812B, 27, COLOR_ORDER>(leds[11], LEDS_PER_STRIP[11]);
    FastLED.addLeds<WS2812B, 32, COLOR_ORDER>(leds[12], LEDS_PER_STRIP[12]);
    FastLED.addLeds<WS2812B, 33, COLOR_ORDER>(leds[13], LEDS_PER_STRIP[13]);
    
    FastLED.setBrightness(MAX_BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    
    state.lastUpdateTime = millis();
    Serial.println("Initialization complete. Ready for commands.");
    Serial.println("Commands: f,<0-100> | s | q");
}

// 解析串口命令
void parseCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd.length() == 0) return;
    
    if (cmd.startsWith("f,")) {
        // f,n 命令：设置目标亮度
        int commaPos = cmd.indexOf(',');
        if (commaPos >= 0) {
            String valueStr = cmd.substring(commaPos + 1);
            float targetValue = valueStr.toFloat();
            targetValue = constrain(targetValue, 0.0, 100.0);
            
            // 开始新的过渡
            state.transitionStartValue = state.currentControlValue;
            state.targetControlValue = targetValue;
            state.transitionStartTime = millis();
            state.isTransitioning = true;
            
            Serial.print("OK: Set target to ");
            Serial.println(targetValue);
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
void renderFunctionA(int stripIndex) {
    uint8_t brightnessA = mapControlToBrightnessA(state.currentControlValue);
    int totalLeds = LEDS_PER_STRIP[stripIndex];
    
    // 动态计算分段位置（a=0%, b=25%, c=75%, d=100%）
    int segAStart = 0;                              // a = 0%
    int segBStart = (int)(totalLeds * 0.25);        // b = 25%
    int segCStart = (int)(totalLeds * 0.75);        // c = 75%
    int segDEnd = totalLeds;                       // d = 100%
    
    // 区域1：a→b (0% - 25%)
    for (int i = segAStart; i < segBStart; i++) {
        if (i % 3 == 0) {  // 每隔2颗亮1颗（每3颗中第1颗）
            leds[stripIndex][i] = CHSV(0, 0, brightnessA); // 白色，HSV模式
        } else {
            leds[stripIndex][i] = CRGB::Black;
        }
    }
    
    // 区域2：c→d (75% - 100%)
    for (int i = segCStart; i < segDEnd; i++) {
        if (i % 3 == 0) {  // 每隔2颗亮1颗
            leds[stripIndex][i] = CHSV(0, 0, brightnessA);
        } else {
            leds[stripIndex][i] = CRGB::Black;
        }
    }
}

// 渲染功能B（虚拟位置系统）
void renderFunctionB(int stripIndex) {
    uint8_t baseBrightnessB = mapControlToBrightnessB(state.currentControlValue);
    VirtualPosRange range = STRIP_VIRTUAL_MAP[stripIndex];
    int totalLeds = LEDS_PER_STRIP[stripIndex];
    
    // 动态计算分段位置（b=25%, c=75%）
    int bStart = (int)(totalLeds * 0.25);  // b = 25%
    int cEnd = (int)(totalLeds * 0.75);    // c = 75%
    int bSegmentLength = cEnd - bStart;
    
    for (int i = bStart; i < cEnd; i++) {
        // 将灯珠索引映射到虚拟位置
        float localPos;
        if (range.start == range.end) {
            // 单个虚拟位置（如灯带5显示位置1）
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
        leds[stripIndex][i] = CHSV(160, 255, finalBrightness); // 青色，HSV模式
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
    
    // 4. 渲染所有灯带
    for (int s = 0; s < NUM_STRIPS; s++) {
        int totalLeds = LEDS_PER_STRIP[s];
        // 先清空
        for (int i = 0; i < totalLeds; i++) {
            leds[s][i] = CRGB::Black;
        }
        
        // 渲染功能A
        renderFunctionA(s);
        
        // 渲染功能B
        renderFunctionB(s);
    }
    
    // 5. 更新显示
    FastLED.show();
}
