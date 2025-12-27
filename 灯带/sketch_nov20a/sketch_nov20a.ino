#include <FastLED.h>

// ================= 配置区域 =================

// 1. 灯带参数
#define NUM_STRIPS      14      // 接口数量
#define LEDS_PER_STRIP  500     // 每个接口的灯珠数
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define BRIGHTNESS      128     // 总体亮度 (注意供电! 7000颗灯需要巨大电流)

// 2. 动画时间参数
#define GROUP_INTERVAL_MS   500   // 同一接口内，组与组切换的间隔 (0.5秒)
#define STRIP_START_DELAY   1000  // 不同接口之间，启动的间隔 (1秒)
#define TRAIL_DECAY         40    // 拖尾消失速度 (数值越大消失越快，范围0-255)

// 3. 定义14个接口的引脚 (请根据实际接线修改)
// 建议避开 input-only 引脚 (34-39) 和 Flash 引脚 (6-11)
const uint8_t STRIP_PINS[NUM_STRIPS] = {
    13, 12, 14, 27, 26, 25, 33, 32, 15, 2, 4, 16, 17, 5
};

// ===========================================

// 灯珠数据二维数组：[第几条带][第几颗灯]
CRGB leds[NUM_STRIPS][LEDS_PER_STRIP];

// 分组结构体
struct LedGroup {
    int start;
    int end;
};

// 分组定义 (注意：这里使用你提供的原始编号，代码里会自动 -1 适配数组)
const LedGroup groups[] = {
    { 2, 12 }, { 22, 45 }, { 55, 85 },
    { 95, 125 }, { 135, 166 }, { 176, 210 },
    { 220, 255 }, { 265, 296 }, { 306, 336 },
    { 346, 375 }, { 385, 410 }, { 420, 440 },
    { 450, 462 }
};
const int NUM_GROUPS = sizeof(groups) / sizeof(groups[0]);

// --- 状态机结构体：用于记录每一条灯带运行到哪一步了 ---
struct StripState {
    bool isActive;              // 是否已经开始运行
    unsigned long startTime;    // 该灯带开始运行的时间点
    int currentGroupIndex;      // 当前亮到第几组了
};

StripState stripStates[NUM_STRIPS]; // 创建14个状态控制器

void setup() {
    Serial.begin(115200);

    // --- 极其暴力的初始化 ---
    // FastLED 的 addLeds 需要编译时确定的引脚，不能用循环变量直接放进去。
    // 这里必须手动列出或者使用这种模板展开的方式。为了代码清晰，我们手动列出。
    
    FastLED.addLeds<LED_TYPE, 13, COLOR_ORDER>(leds[0], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 12, COLOR_ORDER>(leds[1], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 14, COLOR_ORDER>(leds[2], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 27, COLOR_ORDER>(leds[3], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 26, COLOR_ORDER>(leds[4], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 25, COLOR_ORDER>(leds[5], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 33, COLOR_ORDER>(leds[6], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 32, COLOR_ORDER>(leds[7], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 15, COLOR_ORDER>(leds[8], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 2,  COLOR_ORDER>(leds[9], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 4,  COLOR_ORDER>(leds[10], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 16, COLOR_ORDER>(leds[11], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 17, COLOR_ORDER>(leds[12], LEDS_PER_STRIP);
    FastLED.addLeds<LED_TYPE, 5,  COLOR_ORDER>(leds[13], LEDS_PER_STRIP);

    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();

    // 初始化状态
    for (int i = 0; i < NUM_STRIPS; i++) {
        stripStates[i].isActive = false;
        stripStates[i].currentGroupIndex = -1; // -1 表示还没开始第一组
    }
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. 全局拖尾处理 (每一帧都让所有灯变暗一点点)
    // 这里的 60 是期望的帧率，用于平滑衰减
    EVERY_N_MILLISECONDS(20) { 
        for (int s = 0; s < NUM_STRIPS; s++) {
            // fadeToBlackBy 会让颜色变暗，留下残影
            fadeToBlackBy(leds[s], LEDS_PER_STRIP, TRAIL_DECAY);
        }
    }

    // 2. 检查每一条灯带的状态并更新
    for (int s = 0; s < NUM_STRIPS; s++) {
        
        // 逻辑A：判断该灯带是否应该启动了
        // 第 s 条灯带的启动时间 = s * 1000ms
        if (!stripStates[s].isActive) {
            if (currentMillis >= (s * STRIP_START_DELAY)) {
                stripStates[s].isActive = true;
                stripStates[s].startTime = currentMillis; // 记录它真正启动的时间
                Serial.print("Strip "); Serial.print(s); Serial.println(" Started!");
            }
        }

        // 逻辑B：如果灯带已经启动，计算它应该亮哪一组
        if (stripStates[s].isActive) {
            // 计算这条灯带已经运行了多久
            unsigned long timeRunning = currentMillis - stripStates[s].startTime;
            
            // 根据运行时间，算出应该在第几组 (每 500ms 一组)
            int groupIndex = timeRunning / GROUP_INTERVAL_MS;

            // 如果算出来的组号还在范围内
            if (groupIndex < NUM_GROUPS) {
                stripStates[s].currentGroupIndex = groupIndex;
                
                // 获取当前组的范围
                int startLed = groups[groupIndex].start - 1; // 转换为 0-based
                int endLed = groups[groupIndex].end - 1;

                // 点亮当前组 (叠加颜色，配合Fade形成拖尾)
                // 这里使用 Cyan (青色)，你可以换成 CRGB::Red 等
                for (int i = startLed; i <= endLed; i++) {
                    leds[s][i] = CRGB::Cyan; 
                }
            } else {
                // 如果超出了组数，说明跑完了。
                // 你可以在这里决定是循环播放，还是停在最后。
                // 示例：循环播放 (重置开始时间)
                // stripStates[s].startTime = currentMillis; 
            }
        }
    }

    // 3. 更新所有灯带
    FastLED.show();
}
