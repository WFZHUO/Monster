/**
 * @file alg_waveform.cpp
 * @author WFZ
 * @brief PID测试常用波形发生器实现
 * @version 0.0
 * @date 2026-02-03
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_waveform.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 约束到 [__Min,__Max]
 *
 * @param __X 输入
 * @param __Min 最小值
 * @param __Max 最大值
 * @return float 约束后的输出
 */
inline float Class_Waveform::Clamp(float __X, float __Min, float __Max)
{
    if (__X < __Min) return __Min;
    if (__X > __Max) return __Max;
    return __X;
}

/**
 * @brief 把输入包裹约束到 [0,1),即取余1
 *
 * @param __P 输入
 * @return float 约束后的输出
 */
inline float Class_Waveform::Wrap_01(float __P)
{
    if (__P >= 1.0f) __P -= (float)((int)__P);

    if (__P < 0.0f)  __P += (float)((int)(-__P) + 1);
    if (__P >= 1.0f) __P = 0.0f;

    return __P;
}

/**
 * @brief 32位随机数生成器（XORSHIFT）
 *
 * @param __State 随机数状态
 * @return uint32_t 随机数
 */
uint32_t Class_Waveform::XORSHIFT32(uint32_t &__State)
{
    uint32_t x = __State;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    __State = x;
    return x;
}

/**
 * @brief 初始化波形发生器
 *
 * @param __DT_s 采样周期（秒）
 */
void Class_Waveform::Init(float __DT_s)
{
    Type = Waveform_Type_HOLD;

    DT_s = (__DT_s > 0.0f) ? __DT_s : 0.001f;
    Time_s = 0.0f;

    Output = 0.0f;
    Output_Min = -1e9f;
    Output_Max =  1e9f;

    Amplitude = 0.0f;
    Offset = 0.0f;

    Phase_01 = 0.0f;

    // 默认参数
    Step_Low = 0.0f;
    Step_High = 0.0f;
    Step_T_s = 0.0f;

    Square_Freq_Hz = 1.0f;
    Square_Duty = 0.5f;

    Sine_Freq_Hz = 1.0f;
    Wave_Freq_Hz = 1.0f;

    Ramp_Slope_Per_s = 0.0f;
    Ramp_Period_s = 0.0f;

    Pulse_Low = 0.0f;
    Pulse_High = 1.0f;
    Pulse_Period_s = 1.0f;
    Pulse_Width_s = 0.1f;

    Doublet_A = 1.0f;
    Doublet_T_Pos_s = 0.2f;
    Doublet_T_Zero_s = 0.2f;
    Doublet_T_Neg_s = 0.2f;
    Doublet_Baseline = 0.0f;

    Chirp_A = 1.0f;
    Chirp_F0_Hz = 0.1f;
    Chirp_F1_Hz = 2.0f;
    Chirp_T_s  = 5.0f;

    PRBS_A = 1.0f;
    PRBS_Bit_T_s = 0.05f;
    PRBS_Bit_Timer_s = 0.0f;
    PRBS_LFSR = 0xABCDEu;

    Noise_Amp = 0.1f;
    RNG_State = 0x12345678u;

    Relay_A = 1.0f;
    Relay_Hys = 0.0f;
    Relay_Bias = 0.0f;
    Relay_State = +1.0f;
}

/**
 * @brief 重置时间和相位
 *
 */
void Class_Waveform::Reset()
{
    Time_s = 0.0f;
    Phase_01 = 0.0f;
    PRBS_Bit_Timer_s = 0.0f;
}

/**
 * @brief 获取当前波形类型
 *
 * @return Enum_Waveform_Type 当前波形类型
 */
inline Enum_Waveform_Type Class_Waveform::Get_Type()
{
    return Type;
}

/**
 * @brief 获取当前时间
 *
 * @return float 当前时间（秒）
 */
inline float Class_Waveform::Get_Time_s()
{
    return Time_s;
}

/**
 * @brief 获取当前输出值
 *
 * @return float 当前输出值
 */
inline float Class_Waveform::Get_Output()
{
    return Output;
}

/**
 * @brief 设置输出限幅
 *
 * @param __Output_Min 输出下限
 * @param __Output_Max 输出上限
 */
void Class_Waveform::Set_Limit(float __Output_Min, float __Output_Max)
{
    Output_Min = __Output_Min;
    Output_Max = __Output_Max;
}

/**
 * @brief 设置保持波形
 *
 * @param __Value 保持值
 */
void Class_Waveform::Hold(float __Value)
{
    Type = Waveform_Type_HOLD;
    Output = __Value;
}

/**
 * @brief 设置阶跃波形
 *
 * @param __Low 低电平
 * @param __High 高电平
 * @param __T_Step_s 切换时刻（秒）
 */
void Class_Waveform::Step(float __Low, float __High, float __T_Step_s)
{
    Type = Waveform_Type_STEP;
    Step_Low = __Low;
    Step_High = __High;
    Step_T_s = (__T_Step_s >= 0.0f) ? __T_Step_s : 0.0f;
}

/**
 * @brief 设置方波波形
 *
 * @param __Amplitude 幅值
 * @param __Frequency_Hz 频率（Hz）
 * @param __Offset 偏置
 * @param __Duty_0_1 占空比（0~1）
 */
void Class_Waveform::Square(float __Amplitude, float __Frequency_Hz, float __Offset, float __Duty_0_1)
{
    Type = Waveform_Type_SQUARE;
    Amplitude = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Offset = __Offset;
    Square_Freq_Hz = (__Frequency_Hz > 0.0f) ? __Frequency_Hz : 0.0f;
    Square_Duty = Clamp(__Duty_0_1, 0.0f, 1.0f);
}

/**
 * @brief 设置正弦波波形
 *
 * @param __Amplitude 幅值
 * @param __Frequency_Hz 频率（Hz）
 * @param __Offset 偏置
 * @param __Phase_Deg 初相位（度）
 */
void Class_Waveform::Sine(float __Amplitude, float __Frequency_Hz, float __Offset, float __Phase_Deg)
{
    Type = Waveform_Type_SINE;
    Amplitude = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Offset = __Offset;
    Sine_Freq_Hz = (__Frequency_Hz > 0.0f) ? __Frequency_Hz : 0.0f;

    // 直接把初相位塞进 Phase_01（0~1）
    Phase_01 = Wrap_01(__Phase_Deg / 360.0f);
}

/**
 * @brief 设置三角波波形
 *
 * @param __Amplitude 幅值
 * @param __Frequency_Hz 频率（Hz）
 * @param __Offset 偏置
 */
void Class_Waveform::Triangle(float __Amplitude, float __Frequency_Hz, float __Offset)
{
    Type = Waveform_Type_TRIANGLE;
    Amplitude = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Offset = __Offset;
    Wave_Freq_Hz = (__Frequency_Hz > 0.0f) ? __Frequency_Hz : 0.0f;
}

/**
 * @brief 设置锯齿波波形
 *
 * @param __Amplitude 幅值
 * @param __Frequency_Hz 频率（Hz）
 * @param __Offset 偏置
 */
void Class_Waveform::Saw(float __Amplitude, float __Frequency_Hz, float __Offset)
{
    Type = Waveform_Type_SAW;
    Amplitude = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Offset = __Offset;
    Wave_Freq_Hz = (__Frequency_Hz > 0.0f) ? __Frequency_Hz : 0.0f;
}

/**
 * @brief 设置斜坡波波形
 *
 * @param __Slope_Per_s 斜率（每秒）
 * @param __Offset 偏置
 * @param __Period_s 周期（秒）
 */
void Class_Waveform::Ramp(float __Slope_Per_s, float __Offset, float __Period_s)
{
    Type = Waveform_Type_RAMP;
    Ramp_Slope_Per_s = __Slope_Per_s;
    Offset = __Offset;
    Ramp_Period_s = (__Period_s > 0.0f) ? __Period_s : 0.0f;
}

/**
 * @brief 设置脉冲波波形
 *
 * @param __Low 低电平
 * @param __High 高电平
 * @param __Period_s 周期（秒）
 * @param __Width_s 脉冲宽度（秒）
 */
void Class_Waveform::Pulse(float __Low, float __High, float __Period_s, float __Width_s)
{
    Type = Waveform_Type_PULSE;
    Pulse_Low = __Low;
    Pulse_High = __High;
    Pulse_Period_s = (__Period_s > 0.0f) ? __Period_s : 1.0f;
    Pulse_Width_s = Clamp(__Width_s, 0.0f, Pulse_Period_s);
}

/**
 * @brief 设置双脉冲波波形
 *
 * @param __Amplitude 幅值
 * @param __T_Pos_s 正脉冲宽度（秒）
 * @param __T_Zero_s 零脉冲宽度（秒）
 * @param __T_Neg_s 负脉冲宽度（秒）
 * @param __Baseline 基线
 */
void Class_Waveform::Doublet(float __Amplitude, float __T_Pos_s, float __T_Zero_s, float __T_Neg_s, float __Baseline)
{
    Type = Waveform_Type_DOUBLET;
    Doublet_A = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Doublet_T_Pos_s = (__T_Pos_s > 0.0f) ? __T_Pos_s : 0.0f;
    Doublet_T_Zero_s = (__T_Zero_s > 0.0f) ? __T_Zero_s : 0.0f;
    Doublet_T_Neg_s = (__T_Neg_s > 0.0f) ? __T_Neg_s : 0.0f;
    Doublet_Baseline = __Baseline;
}

/**
 * @brief 设置扫频波波形
 *
 * @param __Amplitude 幅值
 * @param __F0_Hz 起始频率（Hz）
 * @param __F1_Hz 结束频率（Hz）
 * @param __Sweep_T_s 扫频总时长（秒）
 * @param __Offset 偏置
 */
void Class_Waveform::Chirp(float __Amplitude, float __F0_Hz, float __F1_Hz, float __Sweep_T_s, float __Offset)
{
    Type = Waveform_Type_CHIRP;
    Chirp_A = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Chirp_F0_Hz = (__F0_Hz >= 0.0f) ? __F0_Hz : 0.0f;
    Chirp_F1_Hz = (__F1_Hz >= 0.0f) ? __F1_Hz : 0.0f;
    Chirp_T_s = (__Sweep_T_s > 0.0f) ? __Sweep_T_s : 1.0f;
    Offset = __Offset;
}

/**
 * @brief 设置伪随机二进制序列（PRBS）波形
 *
 * @param __Amplitude 幅值
 * @param __Bit_T_s 每个 bit 的持续时间（秒）
 * @param __Seed 随机数种子
 * @param __Offset 偏置
 */
void Class_Waveform::PRBS(float __Amplitude, float __Bit_T_s, uint32_t __Seed, float __Offset)
{
    Type = Waveform_Type_PRBS;
    PRBS_A = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    PRBS_Bit_T_s = (__Bit_T_s > 0.0f) ? __Bit_T_s : 0.05f;
    PRBS_Bit_Timer_s = 0.0f;
    PRBS_LFSR = (__Seed != 0u) ? __Seed : 0xABCDEu;
    Offset = __Offset;
}

/**
 * @brief 设置噪声波波形
 *
 * @param __Noise_Amp 噪声幅值
 * @param __Seed 随机数种子
 * @param __Offset 偏置
 */
void Class_Waveform::Noise(float __Noise_Amp, uint32_t __Seed, float __Offset)
{
    Type = Waveform_Type_NOISE;
    Noise_Amp = (__Noise_Amp >= 0.0f) ? __Noise_Amp : -__Noise_Amp;
    RNG_State = (__Seed != 0u) ? __Seed : 0x12345678u;
    Offset = __Offset;
}

/**
 * @brief 设置继电器波形
 *
 * @param __Amplitude 幅值
 * @param __Hys  hysteresis（ hysteresis ）
 * @param __Bias 偏置
 */
void Class_Waveform::Relay(float __Amplitude, float __Hys, float __Bias)
{
    Type = Waveform_Type_RELAY;
    Relay_A = (__Amplitude >= 0.0f) ? __Amplitude : -__Amplitude;
    Relay_Hys = (__Hys >= 0.0f) ? __Hys : -__Hys;
    Relay_Bias = __Bias;
    Relay_State = +1.0f; // 默认从 +A 开始
}

// -------------------- Update --------------------

float Class_Waveform::Update(float __Feedback)
{
    // 走时间
    Time_s += DT_s;

    float y = Output;

    switch (Type)
    {
        default:
        case Waveform_Type_HOLD:
        {
            y = Output;
        } break;

        case Waveform_Type_STEP:
        {
            y = (Time_s < Step_T_s) ? Step_Low : Step_High;
        } break;

        case Waveform_Type_SQUARE:
        {
            Phase_01 = Wrap_01(Phase_01 + Square_Freq_Hz * DT_s);
            float hi = (Phase_01 < Square_Duty) ? 1.0f : -1.0f;
            y = Offset + Amplitude * hi;
        } break;

        case Waveform_Type_SINE:
        {
            Phase_01 = Wrap_01(Phase_01 + Sine_Freq_Hz * DT_s);
            y = Offset + Amplitude * sinf(2.0f * WAVE_PI * Phase_01);
        } break;

        case Waveform_Type_TRIANGLE:
        {
            Phase_01 = Wrap_01(Phase_01 + Wave_Freq_Hz * DT_s);
            float p = Phase_01;
            float tri = (p < 0.5f) ? (4.0f * p - 1.0f) : (3.0f - 4.0f * p); // [-1,1]
            y = Offset + Amplitude * tri;
        } break;

        case Waveform_Type_SAW:
        {
            Phase_01 = Wrap_01(Phase_01 + Wave_Freq_Hz * DT_s);
            float saw = 2.0f * Phase_01 - 1.0f; // [-1,1)
            y = Offset + Amplitude * saw;
        } break;

        case Waveform_Type_RAMP:
        {
            float tt = Time_s;
            if (Ramp_Period_s > 0.0f)
            {
                while (tt >= Ramp_Period_s) tt -= Ramp_Period_s;
            }
            y = Offset + Ramp_Slope_Per_s * tt;
        } break;

        case Waveform_Type_PULSE:
        {
            float tt = Time_s;
            while (tt >= Pulse_Period_s) tt -= Pulse_Period_s;
            y = (tt < Pulse_Width_s) ? Pulse_High : Pulse_Low;
        } break;

        case Waveform_Type_DOUBLET:
        {
            float t1 = Doublet_T_Pos_s;
            float t2 = t1 + Doublet_T_Zero_s;
            float t3 = t2 + Doublet_T_Neg_s;

            float t = Time_s;
            if(t3 > 0.0f)
            {
                t = fmod(t,t3);
                if(t < 0.0f)
                {
                    t += t3;
                }
            }

            if (t < t1)          y = Doublet_Baseline + Doublet_A;
            else if (t < t2)     y = Doublet_Baseline;
            else                 y = Doublet_Baseline - Doublet_A;
        } break;

        case Waveform_Type_CHIRP:
        {
            float tt = Time_s;
            float f = Chirp_F1_Hz;
            if (tt < Chirp_T_s)
            {
                float k = tt / Chirp_T_s;
                f = Chirp_F0_Hz + (Chirp_F1_Hz - Chirp_F0_Hz) * k;
            }
            Phase_01 = Wrap_01(Phase_01 + f * DT_s);
            y = Offset + Chirp_A * sinf(2.0f * WAVE_PI * Phase_01);
        } break;

        case Waveform_Type_PRBS:
        {
            PRBS_Bit_Timer_s += DT_s;
            if (PRBS_Bit_Timer_s >= PRBS_Bit_T_s)
            {
                PRBS_Bit_Timer_s -= PRBS_Bit_T_s;

                // 32-bit Galois LFSR（常用 tap）
                uint32_t l = PRBS_LFSR;
                uint32_t lsb = l & 1u;
                l >>= 1;
                if (lsb) l ^= 0x80200003u;
                if (l == 0u) l = 0xABCDEu;
                PRBS_LFSR = l;
            }

            float bit = (PRBS_LFSR & 1u) ? 1.0f : -1.0f;
            y = Offset + PRBS_A * bit;
        } break;

        case Waveform_Type_NOISE:
        {
            uint32_t r = XORSHIFT32(RNG_State);
            float u = (float)(r & 0xFFFFFFu) / (float)0x1000000u; // [0,1)
            float n = 2.0f * u - 1.0f;                            // [-1,1)
            y = Offset + Noise_Amp * n;
        } break;

        case Waveform_Type_RELAY:
        {
            // Relay：带滞回，避免抖动
            // 当前状态 +1 表示输出 +A；-1 表示输出 -A
            if (Relay_State > 0.0f)
            {
                if (__Feedback > Relay_Hys) Relay_State = -1.0f;
            }
            else
            {
                if (__Feedback < -Relay_Hys) Relay_State = +1.0f;
            }

            y = Relay_Bias + Relay_State * Relay_A;
        } break;
    }

    // 限幅
    Output = Clamp(y, Output_Min, Output_Max);
    return Output;
}
