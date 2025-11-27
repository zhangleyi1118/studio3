#include <Arduino.h>
#include <FastLED.h>

// --- 用户配置区域 ---
#define LED_PIN_1     16    // 第一条灯带的数据引脚
#define LED_PIN_2     17    // 第二条灯带的数据引脚
#define NUM_LEDS      30    // 每条灯带的灯珠数量 (根据实际修改)
#define BRIGHTNESS    120   // 最大亮度 (0-255)
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB   // 如果颜色不对，尝试改为 RGB

// 限制最大电流 (毫安)，保护 ESP32
// 如果是通过 USB 供电给 ESP32 再给灯带，建议设为 400-500mA
#define MAX_POWER_MILLIAMPS 450 

CRGB leds[2][NUM_LEDS]; // 定义一个二维数组，存放两条灯带的数据

void setup() {
  Serial.begin(115200);

  // 初始化两条灯带
  FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(leds[1], NUM_LEDS).setCorrection(TypicalLEDStrip);

  FastLED.setBrightness(BRIGHTNESS);
  
  // 核心功能：自动电源管理
  // 5V电压，限制最大电流为 450mA
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_POWER_MILLIAMPS);
}

void loop() {
  // 1. 让所有灯逐渐变暗 (创建拖尾效果)
  // 参数 60 代表每次循环变暗的速度，数值越大尾巴越短，越省电
  fadeToBlackBy(leds[0], NUM_LEDS, 60);
  fadeToBlackBy(leds[1], NUM_LEDS, 60);

  // 2. 计算光点移动的位置
  // beat8 会生成一个 0-255 的锯齿波，我们把它映射到灯带长度上
  // 参数 30 是每分钟的节拍数 (BPM)，数值越大移动越快
  uint8_t pos = map(beat8(30), 0, 255, 0, NUM_LEDS - 1);

  // 3. 点亮新的位置
  // 使用 HSV 颜色空间，让颜色随时间慢慢变化
  // gHue 是一个全局变量，可以用来做彩虹效果
  static uint8_t gHue = 0; 
  
  // 给两条灯带当前位置赋值
  leds[0][pos] = CHSV(gHue, 200, 255);
  leds[1][pos] = CHSV(gHue + 20, 200, 255); // 第二条稍微错开一点颜色

  // 4. 更新颜色循环
  EVERY_N_MILLISECONDS(20) { gHue++; }

  // 5. 发送数据并应用电源限制
  FastLED.show();
  
  // 控制刷新率，大概 60FPS
  FastLED.delay(1000 / 60); 
}