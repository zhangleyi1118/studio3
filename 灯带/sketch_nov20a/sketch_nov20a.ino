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

#ifndef PI
#define PI 3.14159265358979323846
#endif

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

// 潮汐桥粒子系统参数
#define MAX_PARTICLES         30      // 最大粒子数量

// 潮汐桥区域定义
#define CORE_START_U          0.82    // 亮核区起始位置（末端18%）
#define ACCEL_START_U         0.70    // 加速区起始位置
#define FLASH_START_U         0.88    // 闪爆区起始位置

// 底色渐变关键点（RGB值）
#define GRADIENT_RED_R        255     // u=0.00: 深红
#define GRADIENT_RED_G        20
#define GRADIENT_RED_B        0
#define GRADIENT_PINK_R       255     // u=0.55: 粉白
#define GRADIENT_PINK_G       140
#define GRADIENT_PINK_B       120
#define GRADIENT_WARM_R       255     // u=0.80: 暖白
#define GRADIENT_WARM_G       220
#define GRADIENT_WARM_B       200
#define GRADIENT_COLD_R       220     // u=1.00: 冷白略蓝
#define GRADIENT_COLD_G       240
#define GRADIENT_COLD_B       255

// 亮核区颜色
#define CORE_COLOR_R          220
#define CORE_COLOR_G          240
#define CORE_COLOR_B          255

// 白闪颜色
#define FLASH_COLOR_R         235
#define FLASH_COLOR_G         245
#define FLASH_COLOR_B         255

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

// 粒子结构体
struct Particle {
    float x;        // 位置（0..L-1，LED索引）
    float energy;   // 能量（0.5..1.0）
    float hueShift; // 色相偏移（可选，用于随机化）
};

// 潮汐桥控制状态
struct TidalStripState {
    float controlValue;          // 当前控制值n (0-100)
    unsigned long lastUpdateTime; // 上次更新时间（毫秒）
    float globalTime;            // 全局时间（秒，用于所有周期性效果）
    
    // 粒子系统
    Particle particles[30];      // 粒子数组（MAX_PARTICLES=30）
    int particleCount;           // 当前粒子数量
    float lastParticleSpawnTime; // 上次生成粒子的时间（秒）
    
    // 闪爆系统
    float flashIntensity;        // 当前闪爆强度（0-1）
    float flashStartTime;        // 闪爆开始时间（秒）
    float lastFlashCheckTime;    // 上次检查闪爆的时间（秒）
};

TidalStripState tidal_strip1_state = {
    .controlValue = 50.0,  // 默认值50，自主运行
    .lastUpdateTime = 0,
    .globalTime = 0.0,
    .particleCount = 0,
    .lastParticleSpawnTime = 0.0,
    .flashIntensity = 0.0,
    .flashStartTime = 0.0,
    .lastFlashCheckTime = 0.0
};

TidalStripState tidal_strip2_state = {
    .controlValue = 50.0,  // 默认值50，自主运行
    .lastUpdateTime = 0,
    .globalTime = 0.0,
    .particleCount = 0,
    .lastParticleSpawnTime = 0.0,
    .flashIntensity = 0.0,
    .flashStartTime = 0.0,
    .lastFlashCheckTime = 0.0
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

// ================= 工具函数 =================

// 线性插值（重命名以避免与C++标准库冲突）
float lerpFloat(float a, float b, float t) {
    return a + (b - a) * constrain(t, 0.0, 1.0);
}

// 平滑步进函数
float smoothstep(float edge0, float edge1, float x) {
    x = constrain((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// 随机浮点数
float randomFloat(float min, float max) {
    return min + (max - min) * (random(0, 10000) / 10000.0);
}

// 将输入的n (0-100) 映射到内部值 (5-65)
// n=0 → 5, n=100 → 65，中间线性渐变
float mapControlValue(float n) {
    return 1.0 + (n / 100.0) * 60.0;
}

// ================= 潮汐桥函数 =================

// 获取底色渐变颜色
CRGB getGradientColor(float u) {
    u = constrain(u, 0.0, 1.0);
    CRGB color;
    
    if (u <= 0.55) {
        // u=0.00(深红) → u=0.55(粉白)
        float t = u / 0.55;
        color.r = (uint8_t)lerpFloat(GRADIENT_RED_R, GRADIENT_PINK_R, t);
        color.g = (uint8_t)lerpFloat(GRADIENT_RED_G, GRADIENT_PINK_G, t);
        color.b = (uint8_t)lerpFloat(GRADIENT_RED_B, GRADIENT_PINK_B, t);
    } else if (u <= 0.80) {
        // u=0.55(粉白) → u=0.80(暖白)
        float t = (u - 0.55) / 0.25;
        color.r = (uint8_t)lerpFloat(GRADIENT_PINK_R, GRADIENT_WARM_R, t);
        color.g = (uint8_t)lerpFloat(GRADIENT_PINK_G, GRADIENT_WARM_G, t);
        color.b = (uint8_t)lerpFloat(GRADIENT_PINK_B, GRADIENT_WARM_B, t);
    } else {
        // u=0.80(暖白) → u=1.00(冷白)
        float t = (u - 0.80) / 0.20;
        color.r = (uint8_t)lerpFloat(GRADIENT_WARM_R, GRADIENT_COLD_R, t);
        color.g = (uint8_t)lerpFloat(GRADIENT_WARM_G, GRADIENT_COLD_G, t);
        color.b = (uint8_t)lerpFloat(GRADIENT_WARM_B, GRADIENT_COLD_B, t);
    }
    
    return color;
}

// 生成新粒子
void spawnParticle(TidalStripState* state, int totalLEDs) {
    if (state->particleCount >= MAX_PARTICLES) return;
    if (state->controlValue <= 0) return;
    
    float n_mapped = mapControlValue(state->controlValue);
    float s = n_mapped / 100.0;
    float rate = 0.2 + 11.8 * pow(s, 1.6);  // 粒子/秒
    
    float timeSinceLastSpawn = state->globalTime - state->lastParticleSpawnTime;
    float spawnInterval = 1.0 / rate;
    
    if (timeSinceLastSpawn >= spawnInterval) {
        Particle* p = &state->particles[state->particleCount];
        p->x = 0.0;  // 从红巨星端开始
        p->energy = randomFloat(0.5, 1.0);
        p->hueShift = randomFloat(-10.0, 10.0);  // 轻微色相偏移
        state->particleCount++;
        state->lastParticleSpawnTime = state->globalTime;
    }
}

// 更新粒子位置
void updateParticles(TidalStripState* state, int totalLEDs, float deltaTime) {
    if (state->controlValue <= 0) {
        state->particleCount = 0;  // 清空所有粒子
        return;
    }
    
    float n_mapped = mapControlValue(state->controlValue);
    float s = n_mapped / 100.0;
    float baseSpeed = lerpFloat(10.0, 260.0, s);  // LEDs/秒
    
    // 更新每个粒子的位置
    for (int i = state->particleCount - 1; i >= 0; i--) {
        Particle* p = &state->particles[i];
        
        // 计算加速区速度倍增
        float u = p->x / (totalLEDs - 1);
        float accelMask = smoothstep(ACCEL_START_U, 1.0, u);
        float speedMul = 1.0 + accelMask * lerpFloat(0.0, 2.5, s);
        
        // 更新位置
        p->x += baseSpeed * speedMul * deltaTime;
        
        // 移除超出范围的粒子（交换到末尾并减少计数）
        if (p->x >= totalLEDs) {
            // 交换到末尾
            if (i < state->particleCount - 1) {
                state->particles[i] = state->particles[state->particleCount - 1];
            }
            state->particleCount--;
        }
    }
}

// 渲染粒子到LED数组
void renderParticles(CRGB* leds, int totalLEDs, TidalStripState* state) {
    if (state->particleCount == 0) return;
    
    float n_mapped = mapControlValue(state->controlValue);
    float s = n_mapped / 100.0;
    float tailLength = lerpFloat(6.0, 40.0, s);
    float pAmp = lerpFloat(0.15, 1.00, s);
    
    for (int i = 0; i < totalLEDs; i++) {
        CRGB particleContribution = CRGB::Black;
        
        for (int j = 0; j < state->particleCount; j++) {
            Particle* p = &state->particles[j];
            float dist = fabs((float)i - p->x);
            
            // 粒子尾巴：后方更长，前方更短
            float tailLong = tailLength;
            float tailShort = tailLength * 0.25;
            
            float influence = 0.0;
            if (i <= p->x) {
                // 后方（较长尾巴）
                if (dist < tailLong) {
                    influence = 1.0 - (dist / tailLong);
                }
            } else {
                // 前方（较短尾巴）
                if (dist < tailShort) {
                    influence = 1.0 - (dist / tailShort);
                }
            }
            
            if (influence > 0.0) {
                // 获取粒子位置对应的底色
                float u_p = p->x / (totalLEDs - 1);
                CRGB particleColor = getGradientColor(u_p);
                
                // 应用粒子能量和幅度
                float contribution = pAmp * p->energy * influence;
                particleContribution.r = min(255, particleContribution.r + (uint8_t)(particleColor.r * contribution));
                particleContribution.g = min(255, particleContribution.g + (uint8_t)(particleColor.g * contribution));
                particleContribution.b = min(255, particleContribution.b + (uint8_t)(particleColor.b * contribution));
            }
        }
        
        // 累加到LED
        leds[i].r = min(255, leds[i].r + particleContribution.r);
        leds[i].g = min(255, leds[i].g + particleContribution.g);
        leds[i].b = min(255, leds[i].b + particleContribution.b);
    }
}

// 计算亮核区呼吸效果
float calculateCoreBoost(float u, float n, float t) {
    if (u < CORE_START_U) return 0.0;
    
    float n_mapped = mapControlValue(n);
    float s = n_mapped / 100.0;
    float coreFreq = lerpFloat(0.15, 1.20, s);
    float coreAmp = lerpFloat(0.10, 0.60, s);
    float coreBreath = 0.5 + 0.5 * sin(2.0 * PI * coreFreq * t);
    
    float coreMask = smoothstep(CORE_START_U, 1.0, u);
    return coreMask * coreAmp * coreBreath;
}

// 计算潮汐拉伸波
float calculateStretchWave(float u, float n, float t) {
    float n_mapped = mapControlValue(n);
    float s = n_mapped / 100.0;
    float waveFreq = lerpFloat(0.03, 0.20, s);
    float waveSpeed = lerpFloat(0.2, 2.0, s);  // u/秒
    float waveAmp = lerpFloat(0.00, 0.25, s);
    
    float wave = sin(2.0 * PI * (waveFreq * t - waveSpeed * (1.0 - u)));
    float coreMask = smoothstep(CORE_START_U, 1.0, u);
    float stretch = 1.0 + waveAmp * wave * (0.4 + 0.6 * coreMask);
    
    return stretch;
}

// 更新闪爆效果
void updateFlashEffect(TidalStripState* state, float deltaTime) {
    float n_mapped = mapControlValue(state->controlValue);
    
    // 更新现有闪爆的衰减
    if (state->flashIntensity > 0.0) {
        float elapsed = state->globalTime - state->flashStartTime;
        float decayRate = 5.0;  // 衰减速度（每秒衰减5倍）
        state->flashIntensity = exp(-decayRate * elapsed);
        if (state->flashIntensity < 0.01) {
            state->flashIntensity = 0.0;
        }
    }
    
    // 检查是否触发新闪爆（仅当 n_mapped > 85）
    if (n_mapped > 85.0 && state->flashIntensity == 0.0) {
        float timeSinceLastCheck = state->globalTime - state->lastFlashCheckTime;
        
        if (timeSinceLastCheck >= 1.0) {  // 每秒检查一次
            float p = pow((n_mapped - 85.0) / 15.0, 2.0) * 0.6;  // 触发概率
            if (randomFloat(0.0, 1.0) < p) {
                state->flashIntensity = 1.0;
                state->flashStartTime = state->globalTime;
            }
            state->lastFlashCheckTime = state->globalTime;
        }
    }
}

// 获取闪爆贡献
CRGB getFlashContribution(float u, TidalStripState* state) {
    if (u < FLASH_START_U || state->flashIntensity <= 0.0) {
        return CRGB::Black;
    }
    
    float flashMask = smoothstep(FLASH_START_U, 1.0, u);
    float intensity = state->flashIntensity * flashMask;
    
    CRGB flash;
    flash.r = (uint8_t)(FLASH_COLOR_R * intensity);
    flash.g = (uint8_t)(FLASH_COLOR_G * intensity);
    flash.b = (uint8_t)(FLASH_COLOR_B * intensity);
    
    return flash;
}

// 计算潮汐桥LED的颜色和亮度（新版本）
CRGB calculateTidalLEDColor(int ledIndex, int totalLEDs, TidalStripState* state) {
    float u = (float)ledIndex / (totalLEDs - 1);
    u = constrain(u, 0.0, 1.0);
    float n_mapped = mapControlValue(state->controlValue);
    float s = n_mapped / 100.0;
    float t = state->globalTime;
    
    // 1. 底色渐变 × baseGain
    CRGB baseColor = getGradientColor(u);
    float baseGain = lerpFloat(0.10, 0.45, s);
    CRGB color;
    color.r = (uint8_t)(baseColor.r * baseGain);
    color.g = (uint8_t)(baseColor.g * baseGain);
    color.b = (uint8_t)(baseColor.b * baseGain);
    
    // 2. + 亮核区颜色 × coreBoost
    float coreBoost = calculateCoreBoost(u, n_mapped, t);
    if (coreBoost > 0.0) {
        color.r = min(255, color.r + (uint8_t)(CORE_COLOR_R * coreBoost));
        color.g = min(255, color.g + (uint8_t)(CORE_COLOR_G * coreBoost));
        color.b = min(255, color.b + (uint8_t)(CORE_COLOR_B * coreBoost));
    }
    
    // 3. × 拉伸波调制
    float stretch = calculateStretchWave(u, n_mapped, t);
    color.r = (uint8_t)constrain(color.r * stretch, 0, 255);
    color.g = (uint8_t)constrain(color.g * stretch, 0, 255);
    color.b = (uint8_t)constrain(color.b * stretch, 0, 255);
    
    // 4. + 闪爆贡献
    CRGB flash = getFlashContribution(u, state);
    color.r = min(255, color.r + flash.r);
    color.g = min(255, color.g + flash.g);
    color.b = min(255, color.b + flash.b);
    
    // 5. 最终clamp
    color.r = constrain(color.r, 0, 255);
    color.g = constrain(color.g, 0, 255);
    color.b = constrain(color.b, 0, 255);
    
    return color;
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
    tidal_strip1_state.globalTime = 0.0;
    tidal_strip2_state.globalTime = 0.0;
    
    // 确保粒子数组和闪爆状态已初始化（结构体定义时已初始化，这里只是确认）
    tidal_strip1_state.particleCount = 0;
    tidal_strip2_state.particleCount = 0;
    tidal_strip1_state.flashIntensity = 0.0;
    tidal_strip2_state.flashIntensity = 0.0;
    
    Serial.println("Tidal Bridge: Particle flow system initialized (default n=50)");
    
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
            
            // 如果设置为0，清空粒子
            if (targetValue == 0) {
                tidal_strip1_state.particleCount = 0;
                tidal_strip2_state.particleCount = 0;
                tidal_strip1_state.flashIntensity = 0.0;
                tidal_strip2_state.flashIntensity = 0.0;
            }
            
            // 计算粒子速度（用于显示）
            float s = targetValue / 100.0;
            float particleSpeed = lerpFloat(10.0, 260.0, s);
            
            Serial.print("OK: Set target to ");
            Serial.print(targetValue);
            Serial.print(" (14 strips speed: ");
            Serial.print(targetValue + 10.0);
            Serial.print(" pos/sec, Tidal particle speed: ");
            Serial.print(particleSpeed, 1);
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
    // 更新两个灯带的状态
    unsigned long currentTime = millis();
    
    // 更新灯带1
    if (tidal_strip1_state.lastUpdateTime == 0) {
        tidal_strip1_state.lastUpdateTime = currentTime;
    }
    float deltaTime1 = (currentTime - tidal_strip1_state.lastUpdateTime) / 1000.0;
    if (deltaTime1 > 1.0) deltaTime1 = 0.016;
    tidal_strip1_state.lastUpdateTime = currentTime;
    tidal_strip1_state.globalTime += deltaTime1;
    
    // 更新灯带2
    if (tidal_strip2_state.lastUpdateTime == 0) {
        tidal_strip2_state.lastUpdateTime = currentTime;
    }
    float deltaTime2 = (currentTime - tidal_strip2_state.lastUpdateTime) / 1000.0;
    if (deltaTime2 > 1.0) deltaTime2 = 0.016;
    tidal_strip2_state.lastUpdateTime = currentTime;
    tidal_strip2_state.globalTime += deltaTime2;
    
    // 更新粒子系统和闪爆效果
    updateParticles(&tidal_strip1_state, TIDAL_STRIP_1_LEDS, deltaTime1);
    spawnParticle(&tidal_strip1_state, TIDAL_STRIP_1_LEDS);
    updateFlashEffect(&tidal_strip1_state, deltaTime1);
    
    updateParticles(&tidal_strip2_state, TIDAL_STRIP_2_LEDS, deltaTime2);
    spawnParticle(&tidal_strip2_state, TIDAL_STRIP_2_LEDS);
    updateFlashEffect(&tidal_strip2_state, deltaTime2);
    
    // 渲染灯带1
    if (tidal_strip1_state.controlValue > 0) {
        // 先清空
        for (int i = 0; i < TIDAL_STRIP_1_LEDS; i++) {
            leds_tidal1[i] = CRGB::Black;
        }
        // 计算每个LED的底色
        for (int i = 0; i < TIDAL_STRIP_1_LEDS; i++) {
            leds_tidal1[i] = calculateTidalLEDColor(i, TIDAL_STRIP_1_LEDS, &tidal_strip1_state);
        }
        // 叠加粒子贡献
        renderParticles(leds_tidal1, TIDAL_STRIP_1_LEDS, &tidal_strip1_state);
    } else {
        // 如果控制值为0，清空灯带
        for (int i = 0; i < TIDAL_STRIP_1_LEDS; i++) {
            leds_tidal1[i] = CRGB::Black;
        }
    }
    
    // 渲染灯带2
    if (tidal_strip2_state.controlValue > 0) {
        // 先清空
        for (int i = 0; i < TIDAL_STRIP_2_LEDS; i++) {
            leds_tidal2[i] = CRGB::Black;
        }
        // 计算每个LED的底色
        for (int i = 0; i < TIDAL_STRIP_2_LEDS; i++) {
            leds_tidal2[i] = calculateTidalLEDColor(i, TIDAL_STRIP_2_LEDS, &tidal_strip2_state);
        }
        // 叠加粒子贡献
        renderParticles(leds_tidal2, TIDAL_STRIP_2_LEDS, &tidal_strip2_state);
    } else {
        // 如果控制值为0，清空灯带
        for (int i = 0; i < TIDAL_STRIP_2_LEDS; i++) {
            leds_tidal2[i] = CRGB::Black;
        }
    }
    
    // 6. 更新显示
    FastLED.show();
}
