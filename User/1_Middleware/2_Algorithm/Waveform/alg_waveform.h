/**
 * @file alg_waveform.h
 * @author WFZ
 * @brief PID测试常用波形发生器（阶跃/方波/正弦/三角/锯齿/斜坡/脉冲/双脉冲/扫频/PRBS/噪声/继电器）
 * @version 0.0
 * @date 2026-02-03
 *
 */

#ifndef ALG_WAVEFORM_H
#define ALG_WAVEFORM_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Exported macros -----------------------------------------------------------*/

#ifndef WAVE_PI
#define WAVE_PI 3.14159265358979323846f
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 波形类型
 *
 */
typedef enum
{
    Waveform_Type_HOLD = 0,   // 恒值
    Waveform_Type_STEP,       // 阶跃
    Waveform_Type_SQUARE,     // 方波
    Waveform_Type_SINE,       // 正弦
    Waveform_Type_TRIANGLE,   // 三角波
    Waveform_Type_SAW,        // 锯齿波
    Waveform_Type_RAMP,       // 斜坡（线性爬升，可循环）
    Waveform_Type_PULSE,      // 脉冲（周期+宽度）
    Waveform_Type_DOUBLET,    // 双脉冲（+A -> 0 -> -A -> 0）
    Waveform_Type_CHIRP,      // 扫频正弦（f0->f1）
    Waveform_Type_PRBS,       // 伪随机二进制序列（±A）
    Waveform_Type_NOISE,      // 近似白噪声
    Waveform_Type_RELAY,      // 继电器激励（需要反馈输入）
} Enum_Waveform_Type;

/**
 * @brief 波形发生器类
 *
 * 使用方法 ：
 *  1) Init(dt)
 *  2) 选一个波形：Square/Sine/Step/...
 *  3) 每个周期调用 Update() 得到输出
 *
 */
class Class_Waveform
{
public:
    void Init(float __DT_s = 0.001f);

    void Reset();

    inline Enum_Waveform_Type Get_Type();
    inline float Get_Time_s();
    inline float Get_Output();

    void Set_Limit(float __Output_Min, float __Output_Max);

    // 恒值：输出 __Value
    void Hold(float __Value);

    // 阶跃：在 t=0 时输出 __Low，在 t=T 时输出 __High
    void Step(float __Low, float __High, float __T_Step_s);

    // 方波：A 持续 t_on -> 0 持续 t_off -> 回 baseline
    void Square(float __Amplitude, float __Frequency_Hz, float __Offset = 0.0f, float __Duty_0_1 = 0.5f);

    // 正弦波
    void Sine(float __Amplitude, float __Frequency_Hz, float __Offset = 0.0f, float __Phase_Deg = 0.0f);

    // 三角波
    void Triangle(float __Amplitude, float __Frequency_Hz, float __Offset = 0.0f);

    // 锯齿波
    void Saw(float __Amplitude, float __Frequency_Hz, float __Offset = 0.0f);

    // 斜坡：y = offset + slope * t；period>0 则周期性回零
    void Ramp(float __Slope_Per_s, float __Offset = 0.0f, float __Period_s = 0.0f);

    // 脉冲：在每个 period 内，前 width 为 high，其余 low
    void Pulse(float __Low, float __High, float __Period_s, float __Width_s);

    // 双脉冲：+A 持续 t_pos -> 0 持续 t_zero -> -A 持续 t_neg -> 回 baseline
    void Doublet(float __Amplitude, float __T_Pos_s, float __T_Zero_s, float __T_Neg_s, float __Baseline = 0.0f);

    // 扫频：线性扫频 f0->f1，T 扫完后保持 f1
    void Chirp(float __Amplitude, float __F0_Hz, float __F1_Hz, float __Sweep_T_s, float __Offset = 0.0f);

    // PRBS：每 bit_T 秒翻一次（伪随机），输出 offset ± amplitude
    void PRBS(float __Amplitude, float __Bit_T_s, uint32_t __Seed = 0xABCDEu, float __Offset = 0.0f);

    // 噪声：输出 offset ± noise_amp
    void Noise(float __Noise_Amp, uint32_t __Seed = 0x12345678u, float __Offset = 0.0f);

    // Relay：继电器激励（用于极限振荡法），需要 Update(feedback)
    void Relay(float __Amplitude, float __Hys, float __Bias = 0.0f);

    // -------------------- 运行：每周期调用一次 --------------------
    // 对普通波形：__Feedback 传 0 即可
    // 对 Relay：__Feedback 传你的反馈量（角度/角速度/速度等）
    float Update(float __Feedback = 0.0f);

private:

    // 当前波形类型
    Enum_Waveform_Type Type;

    // 采样周期 dt(s)
    float DT_s;

    // 运行时间(s)
    float Time_s;

    // 当前输出值
    float Output;

    // 输出限幅下限/上限
    float Output_Min;
    float Output_Max;

    // ---------------- 通用参数（多数波形都用） ----------------
    // 幅值
    float Amplitude;

    // 偏置
    float Offset;

    // 相位(归一化到 0~1)
    float Phase_01;

    // ---------------- Step（阶跃） ----------------
    // 阶跃低电平/高电平
    float Step_Low;
    float Step_High;

    // 阶跃切换时刻（秒）：Time_s < Step_T_s 输出 Low，否则输出 High
    float Step_T_s;

    // ---------------- Square（方波） ----------------
    // 方波频率（Hz）
    float Square_Freq_Hz;

    // 占空比（0~1），例如 0.5 为对称方波
    float Square_Duty;

    // ---------------- Sine（正弦） ----------------
    // 正弦频率（Hz）
    float Sine_Freq_Hz;

    // ---------------- Triangle / Saw（三角波/锯齿波） ----------------
    // 三角/锯齿共用的频率（Hz）
    float Wave_Freq_Hz;

    // ---------------- Ramp（斜坡/爬坡） ----------------
    // 斜率（每秒变化量），单位：输出单位 / s
    float Ramp_Slope_Per_s;

    // 斜坡周期（秒）：到达周期末尾通常会回到起点或按规则重复
    float Ramp_Period_s;

    // ---------------- Pulse（脉冲） ----------------
    // 脉冲低电平/高电平
    float Pulse_Low;
    float Pulse_High;

    // 脉冲周期（秒）
    float Pulse_Period_s;

    // 脉冲宽度（秒），即高电平持续时间
    float Pulse_Width_s;

    // ---------------- Doublet（双脉冲/双激励） ----------------
    // 双脉冲幅值 A（通常先 +A 再 -A）
    float Doublet_A;

    // 正脉冲持续时间（秒）
    float Doublet_T_Pos_s;

    // 中间回到基线的持续时间（秒）
    float Doublet_T_Zero_s;

    // 负脉冲持续时间（秒）
    float Doublet_T_Neg_s;

    // 基线值（双脉冲围绕的中心值）
    float Doublet_Baseline;

    // ---------------- Chirp（扫频/啁啾） ----------------
    // 扫频幅值
    float Chirp_A;

    // 起始频率（Hz）
    float Chirp_F0_Hz;

    // 结束频率（Hz）
    float Chirp_F1_Hz;

    // 扫频总时长（秒）
    float Chirp_T_s;

    // ---------------- PRBS（伪随机二进制序列） ----------------
    // PRBS 输出幅值（通常输出为 ±PRBS_A 或者 0/PRBS_A）
    float PRBS_A;

    // 每个 bit 的持续时间（秒），越短则切换越快
    float PRBS_Bit_T_s;

    // bit 计时器（秒），用于累计到 PRBS_Bit_T_s 后切换到下一 bit
    float PRBS_Bit_Timer_s;

    // LFSR 寄存器状态（线性反馈移位寄存器），用于生成 PRBS 序列
    uint32_t PRBS_LFSR;

    // ---------------- Noise（噪声） ----------------
    // 噪声幅值（一般用于乘以归一化随机数，得到 [-Amp, Amp] 或类似范围）
    float Noise_Amp;

    // 随机数状态（伪随机发生器的 seed/state，每次生成随机数都会更新）
    uint32_t RNG_State;

    // ---------------- Relay（继电/二值非线性激励） ----------------
    // 继电输出幅值 A（输出通常在 +A 和 -A 之间切换）
    float Relay_A;

    // 迟滞（hysteresis）带宽：避免在阈值附近抖动频繁翻转
    float Relay_Hys;

    // 偏置（bias），用于移动继电切换阈值中心
    float Relay_Bias;

    // 当前继电状态（一般存 +A 或 -A，用于带迟滞的状态机）
    float Relay_State; // +A or -A

    inline float Clamp(float __X, float __Min, float __Max);
    inline float Wrap_01(float __P);

    uint32_t XORSHIFT32(uint32_t &__State);

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif
