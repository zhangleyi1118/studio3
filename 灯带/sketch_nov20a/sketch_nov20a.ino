#include <Adafruit_NeoPixel.h>

// --- 关键修改 1：配置参数 ---
#define PIN        13   // 你代码里设置的是 13，请确保信号线接在 GPIO 13
#define NUMPIXELS  60   // 【重要】必须大于 58，否则第 50 和 58 颗灯无法点亮！

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin(); 
  pixels.setBrightness(50); 
  pixels.clear(); 
  pixels.show();
}

void loop() {
  // --- 步骤 1：点亮第 50 颗灯，持续 2 秒 ---
  
  // 先清除之前的所有状态
  pixels.clear();
  
  // 计算机计数从0开始，第50颗的索引是 49
  pixels.setPixelColor(49, pixels.Color(255, 0, 0)); // 红色
  pixels.show();
  
  // 保持亮 2 秒 (2000 毫秒)
  delay(2000);


  // --- 步骤 2：停（全灭） 1 秒 ---
  
  pixels.clear(); // 清除所有颜色
  pixels.show();  // 推送状态，此时灯带全黑
  
  // 保持灭 1 秒
  delay(1000);


  // --- 步骤 3：点亮第 58 颗灯 ---
  
  // 第58颗的索引是 57
  pixels.setPixelColor(57, pixels.Color(0, 255, 0)); // 设为绿色，方便区分
  pixels.show();

  // 【重要】这里建议也加一个延时，否则代码会立刻跳回开头
  // 瞬间执行步骤1的 clear()，导致你几乎看不见第 58 颗灯亮
  delay(2000); 
}