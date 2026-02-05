/**
 * @file alg_pid.cpp
 * @author WangFZhuo
 * @brief PID算法
 * @version 0.0
 * @date 2025-11-15
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param __K_P P值
 * @param __K_I I值
 * @param __K_D D值
 * @param __K_F 加速度前馈
 * @param __I_Out_Max 积分限幅
 * @param __Out_Max 输出限幅
 * @param __D_T 时间片长度
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __K_F,
              float __I_Out_Max, float __D_Out_Max, float __Out_Max,
              float __D_T,
              float __Dead_Zone, 
              float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First,
              PID_Direction __Direction,
              float __D_Filter_Alpha,
              Enum_PID_Zero_Position_Integral_Bleeding __Zero_Position_Integral_Bleeding
            )
{
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    K_F = __K_F;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
    D_Out_Max = __D_Out_Max;
    D_T = __D_T;
    D_Filter_Alpha = __D_Filter_Alpha;
    Dead_Zone = __Dead_Zone;
    I_Variable_Speed_A = __I_Variable_Speed_A;
    I_Variable_Speed_B = __I_Variable_Speed_B;
    I_Separate_Threshold = __I_Separate_Threshold;
    Zero_Position_Integral_Bleeding = __Zero_Position_Integral_Bleeding;
    D_First = __D_First;
    Direction = __Direction;
    if (__D_T <= 0.0f) {
        // 错误处理
        __D_T = 0.001f;
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Class_PID::Get_Integral_Error()
{
    return (Integral_Error);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Class_PID::Get_Out()
{
    return (Out);
}

/**
 * @brief 获取P输出值
 *
 * @return float P输出值
 */
float Class_PID::Get_P_Out()
{
    return (p_out);
}

/**
 * @brief 获取I输出值
 *
 * @return float I输出值
 */
float Class_PID::Get_I_Out()
{
    return (i_out);
}

/**
 * @brief 获取D输出值
 *
 * @return float D输出值
 */
float Class_PID::Get_D_Out()
{
    return (d_out);
}

/**
 * @brief 获取F输出值
 *
 * @return float F输出值
 */
float Class_PID::Get_F_Out()
{
    return (f_out);
}

/**
 * @brief 获取当前误差值
 *
 * @return float 当前误差值
 */
float Class_PID::Get_Error()
{
    return (error);
}

/**
 * @brief 设定PID的P
 *
 * @param __K_P PID的P
 */
void Class_PID::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定PID的I
 *
 * @param __K_I PID的I
 */
void Class_PID::Set_K_I(float __K_I)
{
    K_I = __K_I;
}

/**
 * @brief 设定PID的D
 *
 * @param __K_D PID的D
 */
void Class_PID::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

/**
 * @brief 设定前馈
 *
 * @param __K_F 前馈
 */
void Class_PID::Set_K_F(float __K_F)
{
    K_F = __K_F;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param __I_Out_Max 积分限幅, 0为不限制
 */
void Class_PID::Set_I_Out_Max(float __I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param __Out_Max 输出限幅, 0为不限制
 */
void Class_PID::Set_Out_Max(float __Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param __I_Variable_Speed_A 定速内段阈值, 0为不限制
 */
void Class_PID::Set_I_Variable_Speed_A(float __I_Variable_Speed_A)
{
    I_Variable_Speed_A = __I_Variable_Speed_A;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param __I_Variable_Speed_B 变速区间, 0为不限制
 */
void Class_PID::Set_I_Variable_Speed_B(float __I_Variable_Speed_B)
{
    I_Variable_Speed_B = __I_Variable_Speed_B;
}

/**
 * @brief 设定积分分离阈值, 0为不限制
 *
 * @param __I_Separate_Threshold 积分分离阈值, 0为不限制
 */
void Class_PID::Set_I_Separate_Threshold(float __I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
void Class_PID::Set_Target(float __Target)
{
    Target = __Target;
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
void Class_PID::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param __Set_Integral_Error 积分值
 */
void Class_PID::Set_Integral_Error(float __Integral_Error)
{
    Integral_Error = __Integral_Error;
}

/**
 * @brief 设置D项滤波系数
 * @param __D_Filter_Alpha D项低通滤波系数，0~1，越小滤波越强，0表示不使用滤波
 */
void Class_PID::Set_D_Filter_Alpha(float __D_Filter_Alpha) {
    D_Filter_Alpha = __D_Filter_Alpha;
    if (D_Filter_Alpha < 0.0f) D_Filter_Alpha = 0.0f;
    if (D_Filter_Alpha > 1.0f) D_Filter_Alpha = 1.0f;
}

 /**
 * @brief 设置零位积分泄放标志位
 * @param __Zero_Position_Integral_Bleeding 零位积分泄放标志位，用于判断是否在零位
 */
void Class_PID::Set_Zero_Position_Integral_Bleeding(Enum_PID_Zero_Position_Integral_Bleeding __Zero_Position_Integral_Bleeding) {
    Zero_Position_Integral_Bleeding = __Zero_Position_Integral_Bleeding;
}

/**
 * @brief 设置D项限幅
 * @param __D_Out_Max D项限幅，0为不限制
 */
void Class_PID::Set_D_Out_Max(float __D_Out_Max) {
    D_Out_Max = __D_Out_Max;
}

/**
 * @brief PID调整值
 *
 * @return float 输出值
 */
void Class_PID::TIM_Adjust_PeriodElapsedCallback()
{
    // 添加除零保护
    if (D_T <= 0.0f) {
        // 记录错误或使用默认值
        D_T = 0.001f; // 避免除零
    }
    // P输出
    p_out = 0.0f;
    // I输出
    i_out = 0.0f;
    // D输出
    d_out = 0.0f;
    // F输出
    f_out = 0.0f;
    //误差
    error = 0.0f;
    //绝对值误差
    float abs_error;
    //线性变速积分
    float speed_ratio;

    error = Target - Now;
    //根据方向调整误差符号
    if (Direction == PID_REVERSE)
    {
        error = -error;
    }
    abs_error = Math_Abs(error);

    //判断死区
    if (abs_error < Dead_Zone)
    {
        error = 0.0f;
        abs_error = 0.0f;
    }

    //计算p项

    p_out = K_P * error;

    //计算i项

    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)
    {
        //非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        //变速积分
        if (abs_error <= I_Variable_Speed_A)
        {
            //误差小于A，正常积分
            speed_ratio = 1.0f;
        }
        else if (abs_error < I_Variable_Speed_A + I_Variable_Speed_B)
        {
            //误差在A到A+B之间，线性递减
            speed_ratio = 1.0f - (abs_error - I_Variable_Speed_A) / I_Variable_Speed_B;        
        }
        else
        {
            //误差大于A+B，停止积分
            speed_ratio = 0.0f;
        }
    }

    //零位积分泄放
    bool ZPIB_Status = (abs(Target) < Dead_Zone && abs_error < Dead_Zone && Zero_Position_Integral_Bleeding == PID_ZPIB_ENABLE);

    if(!ZPIB_Status){
        //积分分离
        if (I_Separate_Threshold == 0.0f)
        {
            //没有积分分离
            Integral_Error += speed_ratio * D_T * error;

            //如果开启积分限幅，那么在对积分项输出限幅的同时对积分误差也进行限幅，防止积分误差一直增大
            if(K_I > 0.0f && I_Out_Max != 0.0f){
                float integral_error_Max = I_Out_Max / K_I;
                Math_Constrain(&Integral_Error, -integral_error_Max, integral_error_Max);
            }

            i_out = K_I * Integral_Error;
            //积分限幅
            if (I_Out_Max != 0.0f)
            {
                Math_Constrain(&i_out, -I_Out_Max, I_Out_Max);
            }
        }
        else
        {
            //积分分离使能
            if (abs_error < I_Separate_Threshold)
            {
                Integral_Error += speed_ratio * D_T * error;

                //如果开启积分限幅，那么在对积分项输出限幅的同时对积分误差也进行限幅，防止积分误差一直增大
                if(K_I > 0.0f && I_Out_Max != 0.0f){
                    float integral_error_Max = I_Out_Max / K_I;
                    Math_Constrain(&Integral_Error, -integral_error_Max, integral_error_Max);
                }

                i_out = K_I * Integral_Error;
                //积分限幅
                if (I_Out_Max != 0.0f)
                {
                    Math_Constrain(&i_out, -I_Out_Max, I_Out_Max);
                }
            }
            else
            {
                Integral_Error = 0.0f;
                i_out = 0.0f;
            }
        }
    }
    else//为了防止静摩擦力的存在，当电机停止时，残留的积分项输出的力不足以克服积分项让电机旋转，但是残留积分却消耗功率,
    // 但是这个功能也不能盲目开，比如Pitch轴电机为了克服重力，就算速度为0，也要保持一个小的积分项输出，否则会导致电机旋转不稳定
    {
        // 在死区内，不累积新的积分
        // 缓慢释放已有积分
        if (fabsf(Integral_Error) > 0.0001f)
        {
            // 每周期衰减5%
            Integral_Error *= 0.95f;
        }
    }

    //计算d项

    float d_raw = 0.0f;
    if (D_First == PID_D_First_DISABLE) {
        // 没有微分先行
        d_raw = K_D * (error - Pre_Error) / D_T;
    } else {
        // 微分先行使能
        d_raw = K_D * (Out - Pre_Out) / D_T;
    }
    
    // 新增：D项滤波
    if (D_Filter_Alpha > 0.0f) {
        filtered_d_out = D_Filter_Alpha * d_raw + (1.0f - D_Filter_Alpha) * filtered_d_out;
        d_out = filtered_d_out;
    } else {
        d_out = d_raw;
    }

    // 新增：D项限幅
    if (D_Out_Max != 0.0f)
    {
        Math_Constrain(&d_out, -D_Out_Max, D_Out_Max);
    }


    //计算前馈

    f_out = K_F * (Target - Pre_Target) / D_T;

    //计算总共的输出

    Out = p_out + i_out + d_out + f_out;
    //输出限幅
    if (Out_Max != 0.0f)
    {
        Math_Constrain(&Out, -Out_Max, Out_Max);
    }

    //善后工作
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}
/*****************************************************************************/

