/**
 * @file tsk_config_and_callback.cpp
 * @author WFZ
 * @brief 当成main.c来用
 * @version 0.0
 * @date 2025-12-30
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "dvc_serialplot.h"
#include "drv_uart.h"
#include "dvc_buzzer.h"
#include "drv_tim.h"
#include "dvc_dr16.h"
#include "crt_chassis.h"
#include "crt_gimbal.h"
#include "crt_booster.h"
#include "drv_math.h"
#include "alg_waveform.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 串口绘图
Class_Serialplot serialplot;
static char Serialplot_Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
        // 电机调PID
        "pa",
        "ia",
        "da",
        "po",
        "io",
        "do",
        "free",
};

Class_Waveform Waveform;

Class_DR16 dr16;

Class_Chassis Chassis;

Class_Gimbal Gimbal;

Class_Booster Booster;

bool init_finished = false;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 * @param Rx_Buffer CAN1收到的消息
 */
void CAN1_Callback_Function(Struct_CAN_Rx_Buffer * Rx_Buffer){
    switch (Rx_Buffer->Header.StdId)
    {

        case (0x208):
        {
            Gimbal.Motor_Yaw.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x205):
        {
            Gimbal.Motor_Pitch.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x201):
        {
            Chassis.Motor[0].CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x202):
        {
            Chassis.Motor[1].CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x203):
        {
            Chassis.Motor[2].CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x204):
        {
            Chassis.Motor[3].CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}

/**
 * @brief CAN2回调函数
 *
 * @param Rx_Buffer CAN2收到的消息
 */
void CAN2_Callback_Function(Struct_CAN_Rx_Buffer * Rx_Buffer){
    switch (Rx_Buffer->Header.StdId)
    {
        case (0x201):
        {
            Booster.Motor_Friction_Left.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x202):
        {
            Booster.Motor_Friction_Right.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
        case (0x203):
        {
            Booster.Motor_Driver.CAN_RxCpltCallback(Rx_Buffer->Data);
        }
        break;
    }
}

/**
 * @brief UART1串口绘图回调函数
 *
 * @param Buffer UART1收到的消息
 * @param Length 长度
 */
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
				
        case(0):
        {
            Booster.Motor_Driver.PID_Angle.Set_K_P(serialplot.Get_Variable_Value());   
        }
        break;
        case(1):
        {
            Booster.Motor_Driver.PID_Angle.Set_K_I(serialplot.Get_Variable_Value());   
        }
        break;
        case(2):
        {
            Booster.Motor_Driver.PID_Angle.Set_K_D(serialplot.Get_Variable_Value());   
        }
        break;
        case(3):
        {
            Booster.Motor_Driver.PID_Omega.Set_K_P(serialplot.Get_Variable_Value());   
        }
        break;
        case(4):
        {
            Booster.Motor_Driver.PID_Omega.Set_K_I(serialplot.Get_Variable_Value());   
        }
        break;
        case(5):
        {
            Booster.Motor_Driver.PID_Omega.Set_K_D(serialplot.Get_Variable_Value());   
        }
        break;
				
        case(6):
        {

        }
        break;
    }
}

/**
 * @brief UART3遥控器回调函数
 *
 * @param Buffer UART3收到的消息
 * @param Length 长度
 */
void UART_DR16_Call_Back(uint8_t *Buffer, uint16_t Length)  
{
	dr16.UART_RxCpltCallback(Buffer);
}


/**
 * @brief TIM4任务回调函数
 *
 */
void Task1ms_TIM4_Callback()
{    

    //波形发生
    float Waveform_Value = Waveform.Update();

    // 底盘
    //用定时器控制，控制频率为电机回传频率
    Chassis.Set_Gimbal_Angle(Gimbal.Get_Now_Yaw_Angle());
    Chassis.Set_Target_Velocity_X(dr16.Get_Left_Y() * Chassis.Get_Chassis_Max_Speed());
    Chassis.Set_Target_Velocity_Y(-dr16.Get_Left_X() * Chassis.Get_Chassis_Max_Speed());
    Chassis.Set_Target_Omega(dr16.Get_Yaw() * Chassis.Get_Chassis_Max_Omega());
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();

    // 云台Yaw
    //用定时器控制，控制频率为电机回传频率
    //Gimbal.Set_Target_Yaw_Omega(-dr16.Get_Right_X() * 2 * PI );
    Gimbal.Set_Target_Yaw_Omega(-dr16.Get_Mouse_X()*50 * 2 * PI );
	Gimbal.Motor_Yaw.Set_Feedforward_Omega(-Chassis.Get_Now_Omega());

    //计算云台相对底盘角速度，考虑pitch角度的投影
    float gimbal_yaw_imu_omega = (Gimbal.IMU_Gimbal.Get_Gyro_Z()) * cosf(Gimbal.Get_Now_Pitch_Angle()) + (Gimbal.IMU_Gimbal.Get_Gyro_X()) * sinf(Gimbal.Get_Now_Pitch_Angle());
    float gimbal_yaw_extern_omega = gimbal_yaw_imu_omega - Chassis.Get_Now_Omega();
    Gimbal.Motor_Yaw.Set_External_Omega(gimbal_yaw_extern_omega);

    // 云台Pitch
    // 用定时器控制，控制频率为电机回传频率
    //float pitch_omega_cmd_raw = dr16.Get_Right_Y() * 2.0f * PI;   // rad/s，满杆=2π

    float pitch_omega_cmd_raw = -dr16.Get_Mouse_Y()*50 * 2.0f * PI;   // rad/s，满杆=2π
	/**************************斜坡减速，防止pitch轴击打限位***********************************/
    // 获取当前角度和限位
    float current_angle = Gimbal.Get_Now_Pitch_Angle();
    float min_angle = -0.37f;
    float max_angle = 0.75f;

    // 定义减速区域（根据实际需要调整）
    float slowdown_zone = 0.05f; // 距离限位0.05rad（约2.86度）开始减速
    float stop_zone = 0.01f;     // 距离限位0.01rad（约0.57度）完全停止

    // 计算到限位的距离
    float dist_to_min = current_angle - min_angle;
    float dist_to_max = max_angle - current_angle;

    // 应用速度斜坡限制
    float pitch_omega_cmd = pitch_omega_cmd_raw;

    // 如果接近下限
    if (dist_to_min < slowdown_zone) {
        float factor = 1.0f;
        if (dist_to_min < stop_zone) {
            factor = 0.0f;  // 完全停止
        } else {
            factor = (dist_to_min - stop_zone) / (slowdown_zone - stop_zone);
        }
        
        // 只允许向上运动（正速度），向下运动限制为0
        if (pitch_omega_cmd < 0) {
            pitch_omega_cmd = pitch_omega_cmd * factor;
        }
    }

    // 如果接近上限
    if (dist_to_max < slowdown_zone) {
        float factor = 1.0f;
        if (dist_to_max < stop_zone) {
            factor = 0.0f;  // 完全停止
        } else {
            factor = (dist_to_max - stop_zone) / (slowdown_zone - stop_zone);
        }
        
        // 只允许向下运动（负速度），向上运动限制为0
        if (pitch_omega_cmd > 0) {
            pitch_omega_cmd = pitch_omega_cmd * factor;
        }
    }
	/**************************斜坡减速，防止pitch轴击打限位***********************************/
		
    // 1) 积分得到目标角（dt=0.001）
    float tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
    tmp_gimbal_pitch += pitch_omega_cmd * 0.001f;
    Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);

    // 2) 速度前馈
    Gimbal.Motor_Pitch.Set_Feedforward_Omega(pitch_omega_cmd);

	Gimbal.Motor_Pitch.Set_External_Omega(-Gimbal.IMU_Gimbal.Get_Gyro_Y());

    Gimbal.TIM_1ms_Resolution_PeriodElapsedCallback();
    Gimbal.TIM_1ms_Control_PeriodElapsedCallback();

    //发射机构
    // ===== Booster 遥控器逻辑 =====
    /*
    static int last_s2 = 0;

    int s1 = dr16.Get_Left_Switch();   // e.g. UP/MID/DOWN
    int s2 = dr16.Get_Right_Switch();

    if (s1 == DR16_Switch_Status_UP)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
    }
    else
    {
        if (s2 == DR16_Switch_Status_MIDDLE)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
        }
        else if (s2 == DR16_Switch_Status_UP)
        {
            // 单发做“边沿触发”：只有从非MID切到MID那一下打一发
            if (last_s2 != DR16_Switch_Status_UP)
            {
                Booster.Trigger_Spot();
            }
        }
        else if (s2 == DR16_Switch_Status_DOWN)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);
        }
    }
    last_s2 = s2;
    */
// ==================== Booster 输入逻辑（遥控器 / 键鼠双模式） ====================
// 约定：
//  - S1(左拨杆)：UP=全停；MIDDLE=遥控器模式；DOWN=键鼠模式
//  - 遥控器模式：S2(右拨杆) MIDDLE=预热；UP=单发（拨一下打一发）；DOWN=连发
//  - 键鼠模式：进入模式默认预热；鼠标左键：短按=单发，长按=连发（松开停止连发）

static int last_s2 = 0;
static bool mouse_l_pressed = false;
static uint16_t mouse_l_hold_ms = 0;
static bool mouse_l_longpress_latched = false;

// 长按判定阈值（ms）：超过该时间就进入连发
constexpr uint16_t MOUSE_L_LONGPRESS_MS = 200;

int s1 = dr16.Get_Left_Switch();
int s2 = dr16.Get_Right_Switch();

// S1=UP：全停
if (s1 == DR16_Switch_Status_UP)
{
    Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
    last_s2 = s2;
    mouse_l_pressed = false;
    mouse_l_hold_ms = 0;
    mouse_l_longpress_latched = false;
}
// S1=MIDDLE：遥控器模式（保留你原来的逻辑）
else if (s1 == DR16_Switch_Status_MIDDLE)
{
    if (s2 == DR16_Switch_Status_MIDDLE)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
    }
    else if (s2 == DR16_Switch_Status_UP)
    {
        // 单发做“边沿触发”
        if (last_s2 != DR16_Switch_Status_UP)
        {
            Booster.Trigger_Spot();
        }
    }
    else if (s2 == DR16_Switch_Status_DOWN)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);
    }

    last_s2 = s2;

    // 防止从键鼠切回遥控器后还残留长按状态
    mouse_l_pressed = false;
    mouse_l_hold_ms = 0;
    mouse_l_longpress_latched = false;
}
// S1=DOWN：键鼠模式
else if (s1 == DR16_Switch_Status_DOWN)
{
    // 进入键鼠模式默认预热（摩擦轮转，拨弹停）
    if (Booster.Get_Booster_Control_Type() == Booster_Control_Type_DISABLE)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
    }

    Enum_DR16_Key_Status ml = dr16.Get_Mouse_Left_Key();

    // 左键按下沿
    if (ml == DR16_Key_Status_TRIG_FREE_PRESSED)
    {
        mouse_l_pressed = true;
        mouse_l_hold_ms = 0;
        mouse_l_longpress_latched = false;
    }

    // 左键持续按住
    if (ml == DR16_Key_Status_PRESSED && mouse_l_pressed)
    {
        if (mouse_l_hold_ms < 0xFFFF)
        {
            mouse_l_hold_ms++;
        }

        // 长按进入连发（只触发一次）
        if (!mouse_l_longpress_latched && mouse_l_hold_ms >= MOUSE_L_LONGPRESS_MS)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_AUTO);
            mouse_l_longpress_latched = true;
        }
    }

    // 左键松开沿：短按->单发；长按->停连发回到预热
    if (ml == DR16_Key_Status_TRIG_PRESSED_FREE)
    {
        if (!mouse_l_longpress_latched)
        {
            // 单击：打一发
            Booster.Trigger_Spot();
        }
        else
        {
            // 松开：停止连发
            if (Booster.Get_Booster_Control_Type() == Booster_Control_Type_AUTO)
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            }
        }

        mouse_l_pressed = false;
        mouse_l_hold_ms = 0;
        mouse_l_longpress_latched = false;
    }

    // 防呆：不按左键且当前仍是AUTO，强制回到预热
    if (!mouse_l_pressed && ml == DR16_Key_Status_FREE && Booster.Get_Booster_Control_Type() == Booster_Control_Type_AUTO)
    {
        Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
    }

    // 键鼠模式下不使用 s2，避免对 last_s2 造成误触发
    last_s2 = s2;
}

    // 运行 Booster
    Booster.TIM_1ms_Calculate_PeriodElapsedCallback();
    
    TIM_CAN_PeriodElapsedCallback();

    //serialplot调试
    static int interaction_mod2 = 0;
    interaction_mod2++;
    if (interaction_mod2 == 2)
    {
        interaction_mod2 = 0;	

        float mouse_x = -dr16.Get_Mouse_X()*50 * 2 * PI ;
        float Gimbal_Yaw_Now_Omega = Gimbal.Get_Now_Yaw_Omega();

        float mouse_y = -dr16.Get_Mouse_Y()*50 * 2 * PI ;
        float Gimbal_Pitch_Now_Omega = Gimbal.Get_Now_Pitch_Omega();

        //serialplot调试
        serialplot.Set_Data(5,
            &Gimbal_Yaw_Now_Omega,
            &mouse_x,
            &Gimbal_Pitch_Now_Omega,
            &mouse_y,
            &Waveform_Value
        );				

        serialplot.TIM_Write_PeriodElapsedCallback();
        TIM_UART_PeriodElapsedCallback();

    }

}


/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    //CAN总线初始化
	CAN_Init(&hcan1,CAN1_Callback_Function);
    CAN_Init(&hcan2,CAN2_Callback_Function);
    //UART初始化
	UART_Init(&huart1, UART_Serialplot_Call_Back, 100);
	UART_Init(&huart3,UART_DR16_Call_Back,18);
	//TIM初始化
	TIM_Init(&htim4,Task1ms_TIM4_Callback);
    //serialplot初始化
	serialplot.Init(&huart1,20,(char **)Serialplot_Variable_Assignment_List);
    //waveform初始化
    Waveform.Init();
    Waveform.Noise(0.02f);
    //buzzer初始化
	dvc_buzzer_init();
    //dr16初始化s
	dr16.Init(&huart3);
    //chassis初始化
	Chassis.Init();
    //gimbal初始化
	Gimbal.Init();
    //booster初始化
    Booster.Init();
	// 使能调度时钟
	HAL_TIM_Base_Start_IT(&htim4);
    //Laser启动
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	//标记初始化完成
	init_finished = true;
	for(int i=0;i<3;i++)
	{
		dvc_buzzer_SetOn();
		HAL_Delay(100);
		dvc_buzzer_SetOff();
		HAL_Delay(100);
	}


}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{

}


/********************************************************************/
