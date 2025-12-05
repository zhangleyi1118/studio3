#include <Arduino.h>
#include <FastLED.h>

// --- 硬件基础配置 ---
#define LED_PIN_1     13    // 灯带1 (索引0)
#define LED_PIN_2     17    // 灯带2 (索引1)
#define NUM_LEDS_PER_STRIP 30 // 假设每根有30颗，请按实际修改
#define MAX_LEDS      60    // 总灯珠数 (用于内存分配)
#define BRIGHTNESS    120   // 最大亮度
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB

// --- 电源安全配置 ---
// 既然直接接ESP32的5V引脚，电流限制必须严格！
// ESP32开发板的肖特基二极管通常只能承受 500mA 左右
#define MAX_POWER_MILLIAMPS 450 

// --- 逻辑分组配置 ---
#define NUM_GROUPS    5     // 我们将所有灯珠分为 5 组

// 定义物理灯珠对象
CRGB leds[2][NUM_LEDS_PER_STRIP]; 

// 定义虚拟灯珠对象 (用于计算 5 个组的颜色)
CRGB groupColors[NUM_GROUPS];

// --- 映射结构体 ---
struct PixelMap {
  uint8_t stripIdx; // 哪条灯带? (0 代表 PIN 13, 1 代表 PIN 17)
  uint8_t ledIdx;   // 灯带上的第几颗灯珠? (0 到 NUM_LEDS-1)
  uint8_t groupIdx; // 归属于第几组? (0 到 NUM_GROUPS-1)
};

// --- [重要] 在这里配置你的分组 ---
// 这里只是示例，你需要根据实际想要把哪些灯分在一组来修改
// 格式: {灯带索引, 灯珠索引, 归属组号}
const PixelMap pixelMap[] = {
  // === 第 0 组 (假设是两条灯带的开头各3颗) ===
  {0, 0, 0}, {0, 1, 0}, {0, 2, 0}, 
  {1, 0, 0}, {1, 1, 0}, {1, 2, 0},

  // === 第 1 组 ===
  {0, 3, 1}, {0, 4, 1}, {0, 5, 1},
  {1, 3, 1}, {1, 4, 1}, {1, 5, 1},

  // === 第 2 组 ===
  {0, 6, 2}, {0, 7, 2}, {0, 8, 2},
  {1, 6, 2}, {1, 7, 2}, {1, 8, 2},

  // === 第 3 组 ===
  {0, 9, 3}, {0, 10, 3}, {0, 11, 3},
  {1, 9, 3}, {1, 10, 3}, {1, 11, 3},

  // === 第 4 组 ===
  {0, 12, 4}, {0, 13, 4}, {0, 14, 4},
  {1, 12, 4}, {1, 13, 4}, {1, 14, 4},
  
  // 如果你还有更多灯珠需要分组，继续往下写...
  // 没写进这里的灯珠将不会亮
};

// 计算映射表的大小，方便遍历
const int mapSize = sizeof(pixelMap) / sizeof(PixelMap);

void setup() {
  Serial.begin(115200);

  // 初始化物理灯带
  FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(leds[0], NUM_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(leds[1], NUM_LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);

  FastLED.setBrightness(BRIGHTNESS);
  
  // 启用电源管理 (这是保护板子不烧的关键)
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_POWER_MILLIAMPS);
}

void loop() {
  // 1. 逻辑层：处理动画 (针对 5 个组 groupColors 进行计算)
  
  // A. 拖尾效果：让 5 个组的颜色逐渐变暗
  fadeToBlackBy(groupColors, NUM_GROUPS, 60);

  // B. 移动效果：计算光点当前在哪一组
  // beat8(30) 生成波形，map 将其映射到 0 到 4 之间
  uint8_t pos = map(beat8(30), 0, 255, 0, NUM_GROUPS - 1);

  // C. 点亮当前组
  static uint8_t gHue = 0;
  // 这里只改变逻辑颜色，还没显示到灯带上
  groupColors[pos] = CHSV(gHue, 200, 255); 

  // 2. 物理层：将逻辑颜色映射到物理灯珠
  
  // 先清空物理灯带 (可选，防止未分组的灯乱闪，如果你所有灯都分组了这句可去掉)
  // FastLED.clear(); 

  // 遍历映射表，把 groupColors 的颜色搬运到 leds[][]
  for (int i = 0; i < mapSize; i++) {
    uint8_t strip = pixelMap[i].stripIdx; // 哪根线
    uint8_t led   = pixelMap[i].ledIdx;   // 哪颗灯
    uint8_t group = pixelMap[i].groupIdx; // 哪个组

    // 只有当索引有效时才赋值，防止数组越界崩溃
    if (strip < 2 && led < NUM_LEDS_PER_STRIP && group < NUM_GROUPS) {
      leds[strip][led] = groupColors[group];
    }
  }

  // 3. 更新辅助变量
  EVERY_N_MILLISECONDS(20) { gHue++; }

  // 4. 显示与限流
  FastLED.show();
  FastLED.delay(1000 / 60);
}