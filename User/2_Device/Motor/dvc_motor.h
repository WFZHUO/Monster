/**
 * @file dvc_motor.h
 * @author WFZ
 * @brief 大疆电机(GM6020,C620,C610)的配置与操作
 * @version 0.0
 * @date 2025-12-28
 *
 *
 */

#ifndef DVC_MOTOR_H
#define DVC_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"

/* Exported macros -----------------------------------------------------------*/

// RPM换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机状态
 *
 */
typedef enum 
{
    Motor_Status_DISABLE = 0,
    Motor_Status_ENABLE,
}Enum_Motor_Status;

/**
 * @brief 电机的反馈报文ID(枚举量的值其实也就是电机的ID)
 *
 */
typedef enum 
{
    Motor_CAN_ID_UNDEFINED = 0,
    Motor_CAN_ID_0x201,//C620/C610
    Motor_CAN_ID_0x202,//C620/C610
    Motor_CAN_ID_0x203,//C620/C610
    Motor_CAN_ID_0x204,//C620/C610
    Motor_CAN_ID_0x205,//C620/C610,GM6020
    Motor_CAN_ID_0x206,//C620/C610,GM6020
    Motor_CAN_ID_0x207,//C620/C610,GM6020
    Motor_CAN_ID_0x208,//C620/C610,GM6020
    Motor_CAN_ID_0x209,//GM6020
    Motor_CAN_ID_0x20A,//GM6020
    Motor_CAN_ID_0x20B,//GM6020
}Enum_Motor_ID;

/**
 * @brief 电机控制方式
 *
 */
typedef enum 
{
    Motor_Control_Method_VOLTAGE = 0,
    Motor_Control_Method_CURRENT,
    Motor_Control_Method_OMEGA,
    Motor_Control_Method_ANGLE,
}Enum_Motor_Control_Method;


/**
 * @brief 是否开启功率控制, 此时电机须电流作为输出模式, 不可电压控制
 *
 */
/*功率限制逻辑暂时不开启
typedef enum 
{
    Motor_Power_Limit_Status_DISABLE = 0,
    Motor_Power_Limit_Status_ENABLE,
}Enum_Motor_Power_Limit_Status;
*/

/**
 * @brief 电机CAN反馈源数据
 * @note  电机反馈的数据为大端序,后缀有_Reverse表示需要反序
 */
typedef struct 
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
}__attribute__((packed)) Struct_Motor_CAN_Rx_Data;//取消结构体的内存对齐

/**
 * @brief 电机经过处理的数据
 *
 */
typedef struct 
{
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_Temperature;
    //float Now_Power; // 功率限制逻辑暂时不开启
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
}__attribute__((packed)) Struct_Motor_Rx_Data;//取消结构体的内存对齐

/**
 * @brief GM6020电机驱动模式
 *
 */
typedef enum 
{
    GM6020_Driver_Mode_Voltage = 0,
    GM6020_Driver_Mode_Current,
}Enum_GM6020_Driver_Mode;

/**
 * @brief GM6020无刷电机, 单片机控制输出电压/电流
 *
 */
class Class_Motor_GM6020
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    // PID电流环控制
    Class_PID PID_Current;

    void Init(CAN_HandleTypeDef *hcan, Enum_Motor_ID __ID, Enum_Motor_Control_Method __Control_Method = Motor_Control_Method_ANGLE, int32_t __Encoder_Offset = 0,Enum_GM6020_Driver_Mode __Driver_Mode = GM6020_Driver_Mode_Current/*,Enum_Motor_Power_Limit_Status __Power_Limit_Status = Motor_Power_Limit_Status_DISABLE*/, float __Voltage_Max = 24.0f, float __Current_Max = 3.0f);

    inline float Get_Voltage_Max();

    inline float Get_Current_Max();

    /*功率限制逻辑暂时不开启
    inline float Get_Power_K_0();

    inline float Get_Power_K_1();

    inline float Get_Power_K_2();

    inline float Get_Power_A();
    */

    inline float Get_Theoretical_Output_Voltage_Max();

    inline float Get_Theoretical_Output_Current_Max();

    inline Enum_Motor_Status Get_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Encoder();

    inline float Get_Now_Total_Encoder();

    inline float Get_Now_Omega();

    bool External_Omega_Active_Flag = false;
		
    inline float Get_Now_External_Omega();

    inline bool Get_External_Omega_Flag();
    
    inline float Get_Now_Current();

    inline uint8_t Get_Now_Temperature();

    //inline float Get_Now_Power(); // 功率限制逻辑暂时不开启

    //inline float Get_Power_Estimate(); // 功率限制逻辑暂时不开启

    inline Enum_Motor_Control_Method Get_Control_Method();

    inline float Get_Target_Angle();

    inline float Get_Target_Omega();

    inline float Get_Target_Current();

    inline float Get_Target_Voltage();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Current();

    inline float Get_Feedforward_Voltage();

    inline float Get_Power_Factor();

    inline float Get_Out();

    inline void Set_Control_Method(Enum_Motor_Control_Method __Control_Method);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Current(float __Target_Current);

    inline void Set_Target_Voltage(float __Target_Voltage);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Current(float __Feedforward_Current);

    inline void Set_Feedforward_Voltage(float __Feedforward_Voltage);

    //inline void Set_Power_Factor(float __Power_Factor); //功率限制逻辑暂时不开启

    inline void Set_External_Omega(float __External_Omega);

    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();
    
    void TIM_Calculate_PeriodElapsedCallback();

    //void TIM_Power_Limit_After_Calculate_PeriodElapsedCallback(); // 功率限制逻辑暂时不开启

protected:
    //初始化相关变量

    //绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    //收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_ID ID;
    //发送缓存区
    uint8_t *CAN_Tx_Data;
    //编码器偏移
    int32_t Encoder_Offset;
    //驱动模式
    Enum_GM6020_Driver_Mode Driver_Mode;
    // 是否开启功率控制
    //Enum_Motor_Power_Limit_Status Power_Limit_Status; // 功率限制逻辑暂时不开启
    // 最大电压
    float Voltage_Max;
    // 最大电流
    float Current_Max;

    //常量

    /*功率控制相关逻辑暂不开启
    // 功率计算系数
    float Power_K_0 = 0.8130f;
    float Power_K_1 = -0.0005f;
    float Power_K_2 = 6.0021f;
    float Power_A = 1.3715f;
    */

    //一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;
    // 电压到输出的转化系数
    float Voltage_To_Out = 25000.0f / 24.0f;
    // 电流到输出的转化系数
    float Current_To_Out = 16384.0f / 3.0f;
    // 理论最大输出电压
    float Theoretical_Output_Voltage_Max = 24.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 3.0f;
		
    //内部变量

    //当前时刻的电机接收flag
    uint32_t Flag = 0;
    //上一次进入定时器中断检查时的电机接收flag
    uint32_t Pre_Flag = 0;
    //输出量
    float Out = 0.0f;

    //读变量

    //电机状态
    Enum_Motor_Status Motor_Status = Motor_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_Rx_Data Rx_Data;
    //外部速度反馈标志和值
    bool External_Omega_Flag = false;  ///< 是否使用外部速度反馈
    float External_Omega_Feedback = 0.0f;  ///< 外部速度反馈值
    // 下一时刻的功率估计值, W
    //float Power_Estimate; // 功率限制逻辑暂时不开启
    // 外部输入的角速度, rad/s
    float Now_External_Omega = 0.0f;

    //写变量

    //读写变量

    //电机控制方式
    Enum_Motor_Control_Method Control_Method = Motor_Control_Method_ANGLE;
    //目标的角度, rad
    float Target_Angle = 0.0f;
    //目标的速度, rad/s
    float Target_Omega = 0.0f;
    //目标的电流, A
    float Target_Current = 0.0f;  
    //目标的电压, V
    float Target_Voltage = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;
    // 前馈的电压, V
    float Feedforward_Voltage = 0.0f;
    // 功率衰减因数
    //float Power_Factor = 1.0f; // 功率限制逻辑暂时不开启

    //内部函数
    void Data_Process();

    void PID_Calculate();

    //void Power_Limit_Control(); // 功率限制逻辑暂时不开启

    void Output();
};

/**
 * @brief Reusable, C610无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_C610
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(CAN_HandleTypeDef *hcan, Enum_Motor_ID __ID, Enum_Motor_Control_Method __Control_Method = Motor_Control_Method_ANGLE, int32_t __Encoder_Offset = 0, float __Gearbox_Rate = 36.0f, float __Current_Max = 10.0f);

    inline float Get_Current_Max();

    inline float Get_Theoretical_Output_Current_Max();

    inline Enum_Motor_Status Get_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Current();

    inline Enum_Motor_Control_Method Get_Control_Method();

    inline float Get_Target_Angle();

    inline float Get_Now_Encoder();

    inline float Get_Now_Total_Encoder();

    inline float Get_Target_Omega();

    inline float Get_Target_Current();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Current();

    inline float Get_Out();

    inline void Set_Control_Method(Enum_Motor_Control_Method __Control_Method);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Current(float __Target_Current);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Current(float __Feedforward_Current);

    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_ID ID;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;
    //编码器偏移
    int32_t Encoder_Offset;
    // 减速比, 默认带减速箱
    float Gearbox_Rate; 
    // 最大电流
    float Current_Max;

    // 常量

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;

    // 电流到输出的转化系数
    float Current_To_Out = 10000.0f / 10.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 10.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_Status Motor_Status = Motor_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_Rx_Data Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_Control_Method Control_Method = Motor_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的电流, A
    float Target_Current = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;

    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Output();
};

/**
 * @brief Reusable, C620无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_C620
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(CAN_HandleTypeDef *hcan, Enum_Motor_ID __ID, Enum_Motor_Control_Method __Control_Method = Motor_Control_Method_OMEGA, float __Gearbox_Rate = 3591.0f / 187.0f/*, Enum_Motor_Power_Limit_Status __Power_Limit_Status = Motor_Power_Limit_Status_DISABLE*/, float __Current_Max = 20.0f);

    inline float Get_Current_Max();

    /*功率限制逻辑暂时不开启
    inline float Get_Power_K_0();

    inline float Get_Power_K_1();

    inline float Get_Power_K_2();

    inline float Get_Power_A();
    */

    inline float Get_Theoretical_Output_Current_Max();

    inline Enum_Motor_Status Get_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Current();

    inline uint8_t Get_Now_Temperature();

    //inline float Get_Now_Power(); // 功率限制逻辑暂时不开启

    //inline float Get_Power_Estimate(); // 功率限制逻辑暂时不开启

    inline Enum_Motor_Control_Method Get_Control_Method();

    inline float Get_Target_Angle();

    inline float Get_Target_Omega();

    inline float Get_Target_Current();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Current();

    inline float Get_Out();

    inline void Set_Control_Method(Enum_Motor_Control_Method __Control_Method); 

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Current(float __Target_Current);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Current(float __Feedforward_Current);

    //inline void Set_Power_Factor(float __Power_Factor); //功率限制逻辑暂时不开启

    inline void Set_Out(float __Out);

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

    //void TIM_Power_Limit_After_Calculate_PeriodElapsedCallback(); // 功率限制逻辑暂时不开启

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_ID ID;
    // 发送缓存区
    uint8_t *CAN_Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;
    // 是否开启功率控制
    //Enum_Motor_Power_Limit_Status Power_Limit_Status; // 功率限制逻辑暂时不开启
    // 最大电流
    float Current_Max;

    // 常量

    /*功率控制相关逻辑暂不开启
    // 功率计算系数
    float Power_K_0 = 0.8130f;
    float Power_K_1 = -0.0005f;
    float Power_K_2 = 6.0021f;
    float Power_A = 1.3715f;
    */

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;

    // 电流到输出的转化系数
    float Current_To_Out = 16384.0f / 20.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 20.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_Status Motor_Status = Motor_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_Rx_Data Rx_Data;
    // 下一时刻的功率估计值, W
    //float Power_Estimate; // 功率限制逻辑暂时不开启
    
    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_Control_Method Control_Method = Motor_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的电流
    float Target_Current = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;
    // 功率衰减因数
    //float Power_Factor = 1.0f; // 功率限制逻辑暂时不开启

    // 内部函数

    void Data_Process();

    void PID_Calculate();

    //void Power_Limit_Control(); // 功率限制逻辑暂时不开启

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取最大电压, 单位V
 *
 * @return float 最大电压, 单位V
 */
inline float Class_Motor_GM6020::Get_Voltage_Max()
{
    return (Voltage_Max);
}

/**
 * @brief 获取最大电流, 单位A
 *
 * @return float 最大电流, 单位A
 */
inline float Class_Motor_GM6020::Get_Current_Max()
{
    return (Current_Max);
}

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Power_K_0()
{
    return (Power_K_0);
}
*/

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Power_K_1()
{
    return (Power_K_1);
}
*/

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Power_K_2()
{
    return (Power_K_2);
}
*/

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Power_A()
{
    return (Power_A);
}
*/

/**
 * @brief 获取理论最大输出电压, 单位V
 *
 * @return float 理论最大输出电压, 单位V
 */
inline float Class_Motor_GM6020::Get_Theoretical_Output_Voltage_Max()
{
    return (Theoretical_Output_Voltage_Max);
}

/**
 * @brief 获取理论最大输出电流, 单位A
 *
 * @return float 理论最大输出电流, 单位A
 */
inline float Class_Motor_GM6020::Get_Theoretical_Output_Current_Max()
{
    return (Theoretical_Output_Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_Status 电机状态
 */
inline Enum_Motor_Status Class_Motor_GM6020::Get_Status()
{
    return (Motor_Status);
}

/**
 * @brief 获取当前角度, 单位rad
 *
 * @return float 当前角度, 单位rad
 */
inline float Class_Motor_GM6020::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的编码器值
 *
 * @return float 当前的编码器值
 */
inline float Class_Motor_GM6020::Get_Now_Encoder()
{
    return (Rx_Data.Total_Encoder - Encoder_Offset - Rx_Data.Total_Round * Encoder_Num_Per_Round);
}

/**
 * @brief 获取当前的总编码器值
 *
 * @return float 当前的总编码器值
 */
inline float Class_Motor_GM6020::Get_Now_Total_Encoder()
{
    return (Rx_Data.Total_Encoder);
}

/**
 * @brief 获取当前的速度, 单位rad/s
 *
 * @return float 当前的速度, 单位rad/s
 */
inline float Class_Motor_GM6020::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的外部输入角速度, 单位rad/s
 *
 * @return float 当前的外部输入角速度, 单位rad/s
 */
inline float Class_Motor_GM6020::Get_Now_External_Omega()
{
    return (Now_External_Omega);
}

/**
 * @brief 获取是否使用外部输入角速度
 *
 * @return true 是
 * @return false 否
 */
inline bool Class_Motor_GM6020::Get_External_Omega_Flag()
{
    return (External_Omega_Flag);
}

/**
 * @brief 获取当前的电流, 单位A
 *
 * @return float 当前的电流, 单位A
 */
inline float Class_Motor_GM6020::Get_Now_Current()
{
    return (Rx_Data.Now_Current);
}

/**
 * @brief 获取当前的温度, 单位摄氏度
 *
 * @return uint8_t 当前的温度, 单位摄氏度
 */
inline uint8_t Class_Motor_GM6020::Get_Now_Temperature()
{
    return (Rx_Data.Now_Temperature);
}

/**
 * @brief 获取当前的功率, 单位W
 *
 * @return float 当前的功率, 单位W
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Now_Power()
{
    return (Rx_Data.Now_Power);
}
*/

/**
 * @brief 获取下一时刻的功率估计值, 单位W
 *
 * @return float 下一时刻的功率估计值, 单位W
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Power_Estimate()
{
    return (Power_Estimate);
}
*/

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_Control_Method 电机控制方式
 */
inline Enum_Motor_Control_Method Class_Motor_GM6020::Get_Control_Method()
{
    return (Control_Method);
}

/**
 * @brief 获取目标的角度, 单位rad
 *
 * @return float 目标的角度, 单位rad
 */
inline float Class_Motor_GM6020::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, 单位rad/s
 *
 * @return float 目标的速度, 单位rad/s
 */
inline float Class_Motor_GM6020::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的电流, 单位A
 *
 * @return float 目标的电流, 单位A
 */
inline float Class_Motor_GM6020::Get_Target_Current()
{
    return (Target_Current);
}

/**
 * @brief 获取输出量, 单位V
 *
 * @return float 输出量, 单位V
 */
inline float Class_Motor_GM6020::Get_Target_Voltage()
{
    return (Target_Voltage);
}

/**
 * @brief 获取前馈的速度, 单位rad/s
 *
 * @return float 前馈的速度, 单位rad/s
 */
inline float Class_Motor_GM6020::Get_Feedforward_Omega()
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的电流, 单位A
 *
 * @return float 前馈的电流, 单位A
 */
inline float Class_Motor_GM6020::Get_Feedforward_Current()
{
    return (Feedforward_Current);
}

/**
 * @brief 获取前馈的电压, 单位V
 *
 * @return float 前馈的电压, 单位V
 */
inline float Class_Motor_GM6020::Get_Feedforward_Voltage()
{
    return (Feedforward_Voltage);
}

/**
 * @brief 获取功率衰减因数
 *
 * @return float 功率衰减因数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_GM6020::Get_Power_Factor()
{
    return (Power_Factor);
}
*/

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
inline float Class_Motor_GM6020::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
inline void Class_Motor_GM6020::Set_Control_Method(Enum_Motor_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}

/**
 * @brief 设定目标的角度, 单位rad
 *
 * @param __Target_Angle 目标的角度, 单位rad
 */
inline void Class_Motor_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, 单位rad/s
 *
 * @param __Target_Omega 目标的速度, 单位rad/s
 */
inline void Class_Motor_GM6020::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的电流, 单位A
 *
 * @param __Target_Current 目标的电流, 单位A
 */
inline void Class_Motor_GM6020::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief 设定目标的电压, 单位V
 *
 * @param __Target_Voltage 目标的电压, 单位V
 */
inline void Class_Motor_GM6020::Set_Target_Voltage(float __Target_Voltage)
{
    Target_Voltage = __Target_Voltage;
}

/**
 * @brief 设定前馈的速度, 单位rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, 单位rad/s
 */
inline void Class_Motor_GM6020::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的电流, 单位A
 *
 * @param __Feedforward_Current 前馈的电流, 单位A
 */
inline void Class_Motor_GM6020::Set_Feedforward_Current(float __Feedforward_Current)
{
    Feedforward_Current = __Feedforward_Current;
}

/**
 * @brief 设定前馈的电压, 单位V
 *
 * @param __Feedforward_Voltage 前馈的电压, 单位V
 */
inline void Class_Motor_GM6020::Set_Feedforward_Voltage(float __Feedforward_Voltage)
{
    Feedforward_Voltage = __Feedforward_Voltage;
}

/**
 * @brief 设定功率衰减因数
 *
 * @param __Power_Factor 功率衰减因数
 */
/*功率限制逻辑暂时不开启
inline void Class_Motor_GM6020::Set_Power_Factor(float __Power_Factor)
{
    Power_Factor = __Power_Factor;
}
*/

/**
 * @brief 设置外部速度反馈
 *
 * @param __External_Omega 外部速度反馈值 (rad/s)
 */
inline void Class_Motor_GM6020::Set_External_Omega(float __External_Omega)
{
    External_Omega_Feedback = __External_Omega;
    External_Omega_Flag = true;
}

/**
 * @brief 设定输出量
 *
 * @param __Out 输出量
 */
inline void Class_Motor_GM6020::Set_Out(float __Out)
{
    Out = __Out;
}

/**
 * @brief 获取最大电流
 *
 * @return float 最大电流
 */
inline float Class_Motor_C610::Get_Current_Max()
{
    return (Current_Max);
}

/**
 * @brief 获取理论最大输出电流
 *
 * @return float 理论最大输出电流
 */
inline float Class_Motor_C610::Get_Theoretical_Output_Current_Max()
{
    return (Theoretical_Output_Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_Status 电机状态
 */
inline Enum_Motor_Status Class_Motor_C610::Get_Status()
{
    return (Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_C610::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的编码器值
 *
 * @return float 当前的编码器值
 */
inline float Class_Motor_C610::Get_Now_Encoder()
{
    return (Rx_Data.Total_Encoder - Encoder_Offset - Rx_Data.Total_Round * Encoder_Num_Per_Round);
}

/**
 * @brief 获取当前的总编码器值
 *
 * @return float 当前的总编码器值
 */
inline float Class_Motor_C610::Get_Now_Total_Encoder()
{
    return (Rx_Data.Total_Encoder);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_C610::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的电流, A
 *
 * @return 当前的电流, A
 */
inline float Class_Motor_C610::Get_Now_Current()
{
    return (Rx_Data.Now_Current);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_Control_Method 电机控制方式
 */
inline Enum_Motor_Control_Method Class_Motor_C610::Get_Control_Method()
{
    return (Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_C610::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_C610::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的电流
 *
 * @return float 目标的电流
 */
inline float Class_Motor_C610::Get_Target_Current()
{
    return (Target_Current);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_C610::Get_Feedforward_Omega()
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的电流, A
 *
 * @return float 前馈的电流, A
 */
inline float Class_Motor_C610::Get_Feedforward_Current()
{
    return (Feedforward_Current);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
inline float Class_Motor_C610::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Motor_Control_Method 电机控制方式
 */
inline void Class_Motor_C610::Set_Control_Method(Enum_Motor_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_C610::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_C610::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的电流, A
 *
 * @param __Target_Current 目标的电流, A
 */
inline void Class_Motor_C610::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_C610::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的电流, A
 *
 * @param __Feedforward_Current 前馈的电流, A
 */
inline void Class_Motor_C610::Set_Feedforward_Current(float __Feedforward_Current)
{
    Feedforward_Current = __Feedforward_Current;
}

/**
 * @brief 设定输出量
 *
 * @param __Out 输出量
 */
inline void Class_Motor_C610::Set_Out(float __Out)
{
    Out = __Out;
}

/**
 * @brief 获取最大电流
 *
 * @return float 最大电流
 */
inline float Class_Motor_C620::Get_Current_Max()
{
    return (Current_Max);
}

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Power_K_0()
{
    return (Power_K_0);
}
*/

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Power_K_1()
{
    return (Power_K_1);
}
*/    

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Power_K_2()
{
    return (Power_K_2);
}
*/

/**
 * @brief 获取功率计算系数
 *
 * @return float 功率计算系数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Power_A()
{
    return (Power_A);
}
*/

/**
 * @brief 获取理论最大输出电流
 *
 * @return float 理论最大输出电流
 */
inline float Class_Motor_C620::Get_Theoretical_Output_Current_Max()
{
    return (Theoretical_Output_Current_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_C620_Status 电机状态
 */
inline Enum_Motor_Status Class_Motor_C620::Get_Status()
{
    return (Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_C620::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_C620::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的电流, A
 *
 * @return 当前的电流, A
 */
inline float Class_Motor_C620::Get_Now_Current()
{
    return (Rx_Data.Now_Current);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return uint8_t 当前的温度, K
 */
inline uint8_t Class_Motor_C620::Get_Now_Temperature()
{
    return (Rx_Data.Now_Temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Now_Power()
{
    return (Rx_Data.Now_Power);
}
*/

/**
 * @brief 获取下一时刻的功率估计值, W
 *
 * @return float 下一时刻的功率估计值, W
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Power_Estimate()
{
    return (Power_Estimate);
}
*/

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_C620_Control_Method 电机控制方式
 */
inline Enum_Motor_Control_Method Class_Motor_C620::Get_Control_Method()
{
    return (Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_C620::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_C620::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的电流
 *
 * @return float 目标的电流
 */
inline float Class_Motor_C620::Get_Target_Current()
{
    return (Target_Current);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_C620::Get_Feedforward_Omega()
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的电流, A
 *
 * @return float 前馈的电流, A
 */
inline float Class_Motor_C620::Get_Feedforward_Current()
{
    return (Feedforward_Current);
}

/**
 * @brief 获取功率衰减因数
 *
 * @return float 功率衰减因数
 */
/*功率限制逻辑暂时不开启
inline float Class_Motor_C620::Get_Power_Factor()
{
    return (Power_Factor);
}
*/

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
inline float Class_Motor_C620::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Motor_C620_Control_Method 电机控制方式
 */
inline void Class_Motor_C620::Set_Control_Method(Enum_Motor_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_C620::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_C620::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的电流
 *
 * @param __Target_Current 目标的电流
 */
inline void Class_Motor_C620::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_C620::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的电流, A
 *
 * @param __Feedforward_Current 前馈的电流, A
 */
inline void Class_Motor_C620::Set_Feedforward_Current(float __Feedforward_Current)
{
    Feedforward_Current = __Feedforward_Current;
}

/**
 * @brief 设定功率衰减因数
 *
 * @param __Power_Factor 功率衰减因数
 */
/*功率限制逻辑暂时不开启
inline void Class_Motor_C620::Set_Power_Factor(float __Power_Factor)
{
    Power_Factor = __Power_Factor;
}
*/

/**
 * @brief 设定输出量
 *
 * @param __Out 输出量
 */
inline void Class_Motor_C620::Set_Out(float __Out)
{
    Out = __Out;
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
