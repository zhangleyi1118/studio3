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

// ================= 前向声明 =================
struct Particle;
struct TidalStripState;
struct ControlState;
struct Wave;
struct VirtualPosRange;

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
#define PHASE_PERIOD     21.0   // 波相位周期 [0, 21)
#define PHASE_SPEED_SLOW 0.22   // 慢速（虚拟位置单位/秒）
#define PHASE_SPEED_FAST 3.2    // 快速（虚拟位置单位/秒）
#define EVENT_COOLDOWN   1.8    // 事件层冷却时间（秒）
#define TRAIL_RANGE      2.0    // 拖尾前后各2个虚拟位置（保留用于兼容）
#define MAX_WAVES        10     // 最大波数量（同时存在的波）
#define WAVE_INTERVAL_SLOW 10.0  // n=0时波发送间隔（秒）
#define WAVE_INTERVAL_FAST 1.0   // n=100时波发送间隔（秒）

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

// 波结构体
struct Wave {
    float phase;        // 当前相位 [0, 21)
    float speed;        // 波速度（生成时的速度，保持不变）
    float nValue;       // 生成时的n值（用于计算宽度和深度）
    bool active;        // 是否激活
};

// 全局控制状态
struct ControlState {
    float targetControlValue;    // 目标控制值 (0-100)
    float currentControlValue;    // 当前控制值 (0-100)
    unsigned long transitionStartTime;  // 过渡开始时间
    float transitionStartValue;  // 过渡开始时的值
    bool isTransitioning;        // 是否正在过渡
    
    // 波队列系统
    Wave waves[MAX_WAVES];       // 波队列
    int waveCount;               // 当前波数量
    float lastWaveSpawnTime;     // 上次生成波的时间（秒）
    
    bool isPaused;               // 是否暂停
    unsigned long lastUpdateTime; // 上次更新时间（用于计算deltaTime）
    
    // 事件层状态
    float eventIntensity;         // 当前事件强度 (0-1)
    float eventPhase;            // 事件相位（用于扫过效果）
    unsigned long lastEventTime;  // 上次事件时间（用于cooldown）
    float globalTime;            // 全局时间（秒，用于湍流等时间相关效果）
};

ControlState state = {
    .targetControlValue = 100.0,  // 初始状态：整体都亮
    .currentControlValue = 100.0,  // 初始状态：整体都亮
    .transitionStartTime = 0,
    .transitionStartValue = 100.0,
    .isTransitioning = false,
    .waveCount = 0,
    .lastWaveSpawnTime = 0.0,
    .isPaused = false,
    .lastUpdateTime = 0,
    .eventIntensity = 0.0,
    .eventPhase = 0.0,
    .lastEventTime = 0,
    .globalTime = 0.0
};

// 初始化波数组
void initWaves() {
    for (int i = 0; i < MAX_WAVES; i++) {
        state.waves[i].phase = 0.0;
        state.waves[i].speed = 0.0;
        state.waves[i].nValue = 0.0;
        state.waves[i].active = false;
    }
    state.waveCount = 0;
    state.lastWaveSpawnTime = 0.0;
}

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

// 计算循环距离（支持任意周期）
float getCyclicDistance(float pos1, float pos2, float period) {
    float dist = fabs(pos1 - pos2);
    return min(dist, period - dist);
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
    
    // 1. 底色渐变 × baseGain × 位置亮度系数
    CRGB baseColor = getGradientColor(u);
    
    // 整体亮度系数（降低范围，让底色更暗，突出粒子流）
    float baseGain = lerpFloat(0.05, 0.30, s);
    
    // 位置相关的亮度系数（开头0.4，尾部0.8，平滑过渡）
    float positionBrightness = lerpFloat(0.4, 0.8, u);
    
    // 最终底色亮度
    CRGB color;
    color.r = (uint8_t)(baseColor.r * baseGain * positionBrightness);
    color.g = (uint8_t)(baseColor.g * baseGain * positionBrightness);
    color.b = (uint8_t)(baseColor.b * baseGain * positionBrightness);
    
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
    
    // 初始状态：整体都亮（功能A和功能B都显示）
    // 功能A和功能B会在loop中自动渲染，初始值为100.0
    FastLED.show();
    
    state.lastUpdateTime = millis();
    
    // 初始化波队列
    initWaves();
    
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
            
            // 当n值变化时，清空所有旧的波，让新波使用新的n值
            if (targetValue == 0) {
                state.eventIntensity = 0.0;
                state.eventPhase = 0.0;
                state.lastEventTime = 0;
                state.globalTime = 0.0;
                // 清空所有波
                for (int i = 0; i < MAX_WAVES; i++) {
                    state.waves[i].active = false;
                }
                state.waveCount = 0;
                state.lastWaveSpawnTime = 0.0;
            } else {
                // 当n值变化时（即使不是0），也应该清空旧的波
                // 这样新生成的波会使用新的n值
                // 但保留globalTime，让新波可以立即生成
                for (int i = 0; i < MAX_WAVES; i++) {
                    state.waves[i].active = false;
                }
                state.waveCount = 0;
                // 修复：重置为当前时间减去一个间隔，避免立即生成多个波
                float t = constrain(targetValue / 100.0, 0.0, 1.0);
                float waveInterval = lerpFloat(WAVE_INTERVAL_SLOW, WAVE_INTERVAL_FAST, t);
                state.lastWaveSpawnTime = state.globalTime - waveInterval;  // 改为减去间隔，而不是直接等于
            }
            
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

// 生成新波（从0位置）
void spawnWave(ControlState* state) {
    if (state->waveCount >= MAX_WAVES) return;
    
    // 计算当前n值对应的速度（用于新波）
    float n = state->targetControlValue;
    float t = constrain(n / 100.0, 0.0, 1.0);
    float k = smoothstep(0.0, 1.0, t);
    float speed = lerpFloat(PHASE_SPEED_SLOW, PHASE_SPEED_FAST, k);
    
    // 找到空闲的波槽
    for (int i = 0; i < MAX_WAVES; i++) {
        if (!state->waves[i].active) {
            state->waves[i].phase = 0.0;  // 从0位置开始
            state->waves[i].speed = speed;  // 保存生成时的速度
            state->waves[i].nValue = n;     // 保存生成时的n值
            state->waves[i].active = true;
            state->waveCount++;
            
            // 发送波生成信号到串口
            Serial.print("WAVE_SPAWN n=");
            Serial.print(n, 1);
            Serial.print(" speed=");
            Serial.print(speed, 2);
            Serial.print(" phase=0.0");
            Serial.println();
            
            break;
        }
    }
}

// 更新波队列
void updateVirtualPosition() {
    if (state.isPaused) return;
    
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - state.lastUpdateTime) / 1000.0; // 转换为秒
    
    // 防止deltaTime过大（比如系统重启后）
    if (deltaTime > 1.0) {
        deltaTime = 0.016;  // 限制为约60fps的帧时间
    }
    
    state.lastUpdateTime = currentTime;
    state.globalTime += deltaTime;  // 更新全局时间（用于湍流等）
    
    // n映射：t = clamp(n/100, 0..1), k = smoothstep(t)
    float n = state.targetControlValue;  // 使用targetControlValue确保速度立即改变
    float t = constrain(n / 100.0, 0.0, 1.0);
    float k = smoothstep(0.0, 1.0, t);  // 或使用 t*t，这里用smoothstep更平滑
    
    // 速度计算：speed = lerp(slow, fast, k)
    float speed = lerpFloat(PHASE_SPEED_SLOW, PHASE_SPEED_FAST, k);
    
    // 波发送间隔：n=0时10秒，n=100时1秒，中间均匀变化
    float waveInterval = lerpFloat(WAVE_INTERVAL_SLOW, WAVE_INTERVAL_FAST, t);
    
    // 检查是否需要生成新波
    if (state.globalTime - state.lastWaveSpawnTime >= waveInterval) {
        spawnWave(&state);
        state.lastWaveSpawnTime = state.globalTime;
    }
    
    // 更新所有激活的波（每个波使用自己的速度）
    for (int i = 0; i < MAX_WAVES; i++) {
        if (state.waves[i].active) {
            // 使用波自己的速度更新相位
            state.waves[i].phase += state.waves[i].speed * deltaTime;
            
            // 如果波超出范围，移除它
            if (state.waves[i].phase >= PHASE_PERIOD) {
                state.waves[i].active = false;
                state.waveCount--;
            }
        }
    }
}

// 计算端点拉伸后的虚拟位置
float calculateStretchedPos(float basePos, float n) {
    float t = constrain(n / 100.0, 0.0, 1.0);
    float s = smoothstep(0.0, 1.0, t);  // 或 t*t，这里用smoothstep更平滑
    
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

// 计算单个波对虚拟位置的贡献（前20%渐暗，后20%渐亮效果）
float calculateSingleWaveContribution(float pos, float wavePhase, float n) {
    float t = constrain(n / 100.0, 0.0, 1.0);
    float k = smoothstep(0.0, 1.0, t);
    
    // 计算直接距离和方向
    float d = pos - wavePhase;  // 有正负：正数表示在波后，负数表示在波前
    float absD = fabs(d);
    
    // 波宽和深度随n变化
    float width = lerpFloat(1.5, 0.8, k);  // n大时更窄
    float depth = lerpFloat(0.3, 0.85, k);   // n大时更深
    
    // 如果距离超过波宽，认为波已经走过了，贡献为0
    if (absD > width) {
        return 0.0;
    }
    
    // 前20%渐暗，后20%渐亮，中间60%保持最暗
    float frontWidth = width * 0.2;  // 前20%的宽度
    float backWidth = width * 0.2;   // 后20%的宽度
    
    if (d < 0) {
        // 波前（pos < wavePhase）
        if (absD <= frontWidth) {
            // 前20%：从0渐暗到depth
            float t_front = absD / frontWidth;  // 0到1
            float smooth = smoothstep(0.0, 1.0, t_front);
            return depth * smooth;
        } else {
            // 中间60%：保持最暗
            return depth;
        }
    } else {
        // 波后（pos > wavePhase）
        if (absD <= backWidth) {
            // 后20%：从depth渐亮到0
            float t_back = absD / backWidth;  // 0到1
            float smooth = smoothstep(0.0, 1.0, t_back);
            return depth * (1.0 - smooth);  // 从depth到0
        } else {
            // 超过后20%，贡献为0
            return 0.0;
        }
    }
}

// 计算主暗波（所有波的叠加）
float calculateMainDarkWave(float pos, ControlState* state, float n) {
    float totalDarkWave = 0.0;
    
    // 遍历所有激活的波，计算叠加效果
    for (int i = 0; i < MAX_WAVES; i++) {
        if (state->waves[i].active) {
            // 使用波生成时的n值来计算宽度和深度
            float contribution = calculateSingleWaveContribution(pos, state->waves[i].phase, state->waves[i].nValue);
            // 叠加所有波的贡献（取最大值，避免过度叠加）
            totalDarkWave = max(totalDarkWave, contribution);
        }
    }
    
    return totalDarkWave;
}

// 计算回弹过冲效果（所有波的叠加）
float calculateOvershoot(float pos, ControlState* state, float n) {
    float offset = 0.4;  // 回弹位置偏移
    float width2 = 0.6;
    
    float totalOvershoot = 0.0;
    
    // 遍历所有激活的波
    for (int i = 0; i < MAX_WAVES; i++) {
        if (state->waves[i].active) {
            // 使用波生成时的n值来计算回弹增益
            float t = constrain(state->waves[i].nValue / 100.0, 0.0, 1.0);
            float k = smoothstep(0.0, 1.0, t);
            float gain = lerpFloat(0.05, 0.25, k);
            
            // 计算直接距离（不使用循环距离）
            float d = fabs(pos - state->waves[i].phase);
            
            // 限制overshoot的有效范围：只在波后沿的一定范围内有效
            // 如果距离超过 offset + width2*3，认为波已经走过了，overshoot为0
            if (d > offset && d < offset + width2 * 3.0) {
                float overshoot = gain * exp(-((d - offset) * (d - offset)) / (width2 * width2));
                totalOvershoot = max(totalOvershoot, overshoot);  // 取最大值
            }
        }
    }
    
    return totalOvershoot;
}

// 简单的伪噪声函数（value-noise）
float pseudoNoise(float x) {
    // 简单的哈希函数
    float f = sin(x * 12.9898) * 43758.5453;
    return f - floor(f);  // 返回[0,1)
}

// 计算湍流层（空间相关噪声）
float smoothNoise(float pos, float time, float n) {
    float t = constrain(n / 100.0, 0.0, 1.0);
    float k = smoothstep(0.0, 1.0, t);
    
    float freq = 0.8;  // 空间频率
    float rate = lerpFloat(0.1, 1.5, k);  // 时间变化率
    float amp = lerpFloat(0.0, 0.15, k);  // 幅度
    
    float noise = pseudoNoise(pos * freq + time * rate);
    return (noise - 0.5) * amp;  // 中心化并缩放
}

// 更新事件层
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
        if (state->eventPhase >= PHASE_PERIOD) {
            state->eventIntensity = 0.0;
        }
    } else if (now - state->lastEventTime >= (unsigned long)(EVENT_COOLDOWN * 1000)) {
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

// 计算事件层贡献
float calculateEventContribution(float pos, ControlState* state) {
    if (state->eventIntensity <= 0.0) return 0.0;
    
    float d = getCyclicDistance(pos, state->eventPhase, PHASE_PERIOD);
    float width = 0.5;  // 窄暗带
    float pulse = exp(-(d * d) / (width * width));
    
    return -state->eventIntensity * pulse * 0.6;  // 压暗
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

// 渲染功能B（多层冲突效果）
void renderFunctionB(int groupIndex) {
    float n = state.currentControlValue;
    uint8_t baseBrightnessB = mapControlToBrightnessB(n);
    VirtualPosRange range = GROUP_VIRTUAL_MAP[groupIndex];
    int totalLeds = LEDS_PER_GROUP[groupIndex];
    
    // 动态计算分段位置（b=25%, c=75%）
    int bStart = (int)(totalLeds * 0.25);  // b = 25%
    int cEnd = (int)(totalLeds * 0.75);    // c = 75%
    int bSegmentLength = cEnd - bStart;
    
    // 段边界不发光区域（每段开头结尾各3颗灯珠）
    const int SEGMENT_BORDER_LEDS = 3;
    
    for (int i = bStart; i < cEnd; i++) {
        // 将灯珠索引映射到虚拟位置（离散化）
        float localPos;
        int segmentIndex = -1;  // 段索引，用于检测段边界
        
        if (range.start == range.end) {
            // 单个虚拟位置（如组2显示位置1）
            localPos = range.start;
        } else {
            // 多个虚拟位置范围（离散化分段）
            int count = (int)(range.end - range.start + 1);  // 虚拟位置数量
            float progress = (float)(i - bStart) / bSegmentLength;
            int k = constrain((int)floor(progress * count), 0, count - 1);
            localPos = range.start + k;
            segmentIndex = k;  // 记录段索引
        }
        
        // 应用端点拉伸（只对端点组）
        if (range.start == range.end && (range.start == 1.0 || range.start == 20.0)) {
            localPos = calculateStretchedPos(localPos, n);
        }
        
        // 检测是否在段边界（每段开头结尾各3颗灯珠不发光）
        bool isSegmentBorder = false;
        if (range.start != range.end && segmentIndex >= 0) {
            // 计算当前LED在段内的相对位置
            int count = (int)(range.end - range.start + 1);
            float segmentSize = (float)bSegmentLength / count;  // 每段的LED数量
            float segmentStart = segmentIndex * segmentSize;  // 当前段的起始位置（相对于bStart）
            float posInSegment = (float)(i - bStart) - segmentStart;  // LED在段内的位置
            
            // 检查是否在段开头或结尾的3颗灯珠范围内
            if (posInSegment < SEGMENT_BORDER_LEDS || posInSegment >= (segmentSize - SEGMENT_BORDER_LEDS)) {
                isSegmentBorder = true;
            }
        }
        
        // 如果是在段边界，直接设为黑色并跳过后续计算
        if (isSegmentBorder) {
            leds[groupIndex][i] = CRGB::Black;
            continue;
        }
        
        // 1. 底色层
        float brightness = (float)baseBrightnessB / 255.0;
        
        // 2. 主暗波层（高斯脉冲下陷，所有波的叠加）
        float darkWave = calculateMainDarkWave(localPos, &state, n);
        brightness *= (1.0 - darkWave);
        
        // 3. 回弹过冲层（所有波的叠加）
        float overshoot = calculateOvershoot(localPos, &state, n);
        brightness += overshoot;
        
        // 4. 湍流层（空间相关噪声）
        float turbulence = smoothNoise(localPos, state.globalTime, n);
        brightness += turbulence;
        
        // 5. 事件层（冲击波/撕裂瞬变）
        float event = calculateEventContribution(localPos, &state);
        brightness += event;
        
        // 限制亮度范围并应用
        brightness = constrain(brightness, 0.0, 1.0);
        uint8_t finalBrightness = (uint8_t)(brightness * 255.0);
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
    
    // 3. 更新波相位和事件层
    updateVirtualPosition();
    if (!state.isPaused) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - state.lastUpdateTime) / 1000.0;
        if (deltaTime > 1.0) deltaTime = 0.016;
        updateEventLayer(&state, deltaTime);
    }
    
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
