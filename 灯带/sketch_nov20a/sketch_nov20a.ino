/*
 * LED灯带控制系统 - ESP32版本
 * 
 * 分组方案：14根灯带分为5组，同组灯带共用GPIO（硬件并联）
 * - 组1：灯带1-4 → GPIO 2（虚拟位置7-14）
 * - 组2：灯带5 → GPIO 13（虚拟位置1）
 * - 组3：灯带6 → GPIO 14（虚拟位置20）
 * - 组4：灯带7-10 → GPIO 15（虚拟位置15-19）
 * - 组5：灯带11-14 → GPIO 26（虚拟位置2-6）
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

// 4. 定义5个组的GPIO引脚（同组灯带硬件并联到同一GPIO）
const uint8_t GROUP_PINS[NUM_GROUPS] = {
    2,   // 组1：灯带1-4 → GPIO 2
    13,  // 组2：灯带5 → GPIO 13
    14,  // 组3：灯带6 → GPIO 14
    15,  // 组4：灯带7-10 → GPIO 15
    26   // 组5：灯带11-14 → GPIO 26
};

// 5. 组到虚拟位置的映射
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
    Serial.print("Groups: "); Serial.println(NUM_GROUPS);
    Serial.print("Total Strips: "); Serial.println(NUM_STRIPS);
    
    // 初始化拖尾查找表
    setupTrailTable();
    
    // 初始化FastLED（5组，每组一个GPIO）
    // 同组内的灯带通过硬件并联连接到同一个GPIO
    // 这样只需5个GPIO，完全在ESP32的8个RMT通道限制内
    FastLED.addLeds<WS2812B, 2,  COLOR_ORDER>(leds[0], LEDS_PER_GROUP[0]);  // 组1：灯带1-4
    FastLED.addLeds<WS2812B, 13, COLOR_ORDER>(leds[1], LEDS_PER_GROUP[1]);  // 组2：灯带5
    FastLED.addLeds<WS2812B, 14, COLOR_ORDER>(leds[2], LEDS_PER_GROUP[2]);  // 组3：灯带6
    FastLED.addLeds<WS2812B, 15, COLOR_ORDER>(leds[3], LEDS_PER_GROUP[3]);  // 组4：灯带7-10
    FastLED.addLeds<WS2812B, 26, COLOR_ORDER>(leds[4], LEDS_PER_GROUP[4]);  // 组5：灯带11-14
    
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
            leds[groupIndex][i] = CHSV(0, 0, brightnessA); // 白色，HSV模式
        } else {
            leds[groupIndex][i] = CRGB::Black;
        }
    }
    
    // 区域2：c→d (75% - 100%)
    for (int i = segCStart; i < segDEnd; i++) {
        if (i % 3 == 0) {  // 每隔2颗亮1颗
            leds[groupIndex][i] = CHSV(0, 0, brightnessA);
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
        leds[groupIndex][i] = CHSV(160, 255, finalBrightness); // 青色，HSV模式
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
    
    // 5. 更新显示
    FastLED.show();
}
