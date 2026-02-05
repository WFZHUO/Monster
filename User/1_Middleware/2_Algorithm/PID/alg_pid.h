/**
 * @file alg_pid.h
 * @author WangFZhuo
 * @brief PID算法
 * @version 0.0
 * @date 2025-11-15
 *
 *
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PID方向, 取决于控制量与被控制量成正相关还是负相关
 *
 */
typedef enum {
    PID_DIRECT = 0, // 正相关
    PID_REVERSE // 负相关
} PID_Direction;

/**
 * @brief 微分先行
 *
 */
typedef enum {
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
} Enum_PID_D_First; 

/**
 * @brief 零位积分泄放.目标与实际均为0时，释放积分
 *        
 *
 */
typedef enum {
    PID_ZPIB_DISABLE = 0,
    PID_ZPIB_ENABLE,
} Enum_PID_Zero_Position_Integral_Bleeding;

/**
 * @brief Reusable, PID算法
 *
 */
class Class_PID
{
public:
    /**
     * @brief 初始化PID参数
     * @param __K_P              比例增益，需大于等于0
     * @param __K_I              积分增益，需大于等于0
     * @param __K_D              微分增益，需大于等于0
     * @param __K_F              加速度前馈增益，需大于等于0，默认0
     * @param __I_Out_Max        积分限幅，需大于0，默认0表示不限幅
     * @param __D_Out_Max        D项限幅，需大于0，默认0表示不限幅
     * @param __Out_Max          输出限幅，需大于0，默认0表示不限幅
     * @param __D_T              控制周期，需大于0，默认0.001 s
	 * @param __Dead_Zone        死区，需大于0，默认0表示不设置死区
     * @param __I_Variable_Speed_A 变速积分定速段阈值，默认0
     * @param __I_Variable_Speed_B 变速积分变速段区间，默认0,范围：0<__I_Variable_Speed_A<__I_Variable_Speed_B，当__I_Variable_Speed_A = __I_Variable_Speed_B = 0时，代表不变速积分.
	 * @param __I_Separate_Threshold 积分分离阈值，需大于0，默认0表示不开启积分分离
     * @param __D_First          是否启用微分先行，默认禁用
     * @param __Direction        PID方向，默认正相关
     * @param __D_Filter_Alpha   D项低通滤波系数，0~1，越小滤波越强，0表示不使用滤波，默认0
     * @param __Zero_Position_Integral_Bleeding 零位积分泄放，默认禁用
     */
    void Init(float __K_P, float __K_I, float __K_D, float __K_F = 0.0f,
              float __I_Out_Max = 0.0f, float __D_Out_Max = 0.0f, float __Out_Max = 0.0f,
              float __D_T = 0.001f,
              float __Dead_Zone = 0.0f, 
              float __I_Variable_Speed_A = 0.0f,float __I_Variable_Speed_B = 0.0f, float __I_Separate_Threshold = 0.0f,Enum_PID_D_First __D_First = PID_D_First_DISABLE,
              PID_Direction __Direction = PID_DIRECT,
              float __D_Filter_Alpha = 0.0f,
              Enum_PID_Zero_Position_Integral_Bleeding __Zero_Position_Integral_Bleeding = PID_ZPIB_DISABLE
            );

    /*初始化便捷复制模板：
        Init(__K_P, __K_I, __K_D, __K_F,
              __I_Out_Max, __D_Out_Max, __Out_Max,
              __D_T,
              __Dead_Zone, 
              __I_Variable_Speed_A, __I_Variable_Speed_B, __I_Separate_Threshold, __D_First,
              __Direction,
              __D_Filter_Alpha,
              __Zero_Position_Integral_Bleeding
            )
    */

    /**
     * @brief 获取积分误差
     * @return 当前积分误差
     */
    float Get_Integral_Error();

    /**
     * @brief 获取PID输出
     * @return 当前输出值
     */
    float Get_Out();

    /**
     * @brief 获取P输出值
     * @return 当前P输出值
     */
    float Get_P_Out();

    /**
     * @brief 获取I输出值
     * @return 当前I输出值
     */
    float Get_I_Out();

    /**
     * @brief 获取D输出值
     * @return 当前D输出值
     */
    float Get_D_Out();

    /**
     * @brief 获取F输出值
     * @return 当前F输出值
     */
    float Get_F_Out();

    /**
     * @brief 获取当前误差值
     * @return 当前误差值
     */
    float Get_Error();

    /* 各参数单独设置接口，便于运行时动态调整 */
    void Set_K_P(float __K_P);                ///< 设置比例增益
    void Set_K_I(float __K_I);                ///< 设置积分增益
    void Set_K_D(float __K_D);                ///< 设置微分增益
    void Set_K_F(float __K_F);                ///< 设置前馈增益
    void Set_I_Out_Max(float __I_Out_Max);    ///< 设置积分限幅
    void Set_Out_Max(float __Out_Max);        ///< 设置输出限幅
    void Set_I_Variable_Speed_A(float __Variable_Speed_I_A); ///< 设置变速积分定速段阈值
    void Set_I_Variable_Speed_B(float __Variable_Speed_I_B); ///< 设置变速积分变速段区间
    void Set_I_Separate_Threshold(float __I_Separate_Threshold); ///< 设置积分分离阈值
    void Set_Target(float __Target);          ///< 设置目标值
    void Set_Now(float __Now);                ///< 设置当前值
    void Set_Integral_Error(float __Integral_Error); ///< 手动设置积分误差
    void Set_D_Filter_Alpha(float __D_Filter_Alpha); ///< 设置D项低通滤波系数
    void Set_Zero_Position_Integral_Bleeding(Enum_PID_Zero_Position_Integral_Bleeding __Zero_Position_Integral_Bleeding); ///< 设置零位积分泄放标志位
    void Set_D_Out_Max(float __D_Out_Max); ///< 设置D项限幅

    void TIM_Adjust_PeriodElapsedCallback();

protected:
    /* 初始化相关常量 */
    float D_T;                      ///< PID控制周期，单位：秒
    float Dead_Zone;                ///< 死区：误差绝对值小于此值时不输出
    Enum_PID_D_First D_First;       ///< 微分先行开关
    PID_Direction Direction;        ///< PID方向
    Enum_PID_Zero_Position_Integral_Bleeding Zero_Position_Integral_Bleeding = PID_ZPIB_DISABLE; ///< 零位积分泄放

    /* 内部状态变量 */
    float Pre_Now = 0.0f;           ///< 上一周期当前值
    float Pre_Target = 0.0f;        ///< 上一周期目标值
    float Pre_Out = 0.0f;           ///< 上一周期输出值
    float Pre_Error = 0.0f;         ///< 上一周期误差

    /* 输出值（只读外部接口） */
    float Out = 0.0f;               ///< 当前PID输出值
    float p_out = 0.0f;             ///< 当前比例项输出值
    float i_out = 0.0f;             ///< 当前积分项输出值
    float d_out = 0.0f;             ///< 当前微分项输出值
    float f_out = 0.0f;             ///< 当前前馈项输出值
    float error = 0.0f;             ///< 当前误差值

    /* 可调参数（写接口） */
    float K_P = 0.0f;               ///< 比例增益
    float K_I = 0.0f;               ///< 积分增益
    float K_D = 0.0f;               ///< 微分增益
    float K_F = 0.0f;               ///< 前馈增益

    float I_Out_Max = 0.0f;         ///< 积分限幅，0表示不限幅
    float D_Out_Max = 0.0f;         ///< D项限幅，0表示不限幅
    float Out_Max = 0.0f;           ///< 输出限幅，0表示不限幅

    float I_Variable_Speed_A = 0.0f; ///< 变速积分定速段阈值
    float I_Variable_Speed_B = 0.0f; ///< 变速积分变速段区间
    float I_Separate_Threshold = 0.0f; ///< 积分分离阈值，需为正数

    float D_Filter_Alpha = 0.0f;  ///< D项滤波系数

    float Target = 0.0f;            ///< 当前目标值
    float Now = 0.0f;               ///< 当前测量值

    /* 读写变量 */
    float Integral_Error = 0.0f;    ///< 积分误差累计值
    float filtered_d_out = 0.0f;  ///< 滤波后的D项输出
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/*
模板：
Class_PID XXX_PID;

XXX_PID.Init(0,0,0,0,0,0,0.001);//P, I, D, F, I_Out_Max, Out_Max 均为0，D_T 为 0.001s

假设这是一个1ms执行一次的函数{

		XXX_PID.Set_Target(Target_XXX);
		XXX_PID.Set_Now(Now_XXX);
		XXX_PID.TIM_Adjust_PeriodElapsedCallback();
		Output = XXX_PID.Get_Out();//然后这个Output被你拿去用

}

*/


/*****************************************************************************/

