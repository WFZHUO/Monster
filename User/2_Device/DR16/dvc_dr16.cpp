/**
 * @file DR16.cpp
 * @author wfz 
 * @brief DR16遥控器
 * @version 0.0
 * @date 2026-1-4
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控器DR16初始化
 *
 * @param huart 指定的UART
 */
void Class_DR16::Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &DBUS_Manage_Object;
    }
    // 初始化数据为0
    memset(&Data, 0, sizeof(Data));
    memset(&Pre_UART_Rx_Data, 0, sizeof(Pre_UART_Rx_Data));
    
    // 设置开关和按键的默认状态
    Data.Left_Switch = DR16_Switch_Status_UP;
    Data.Right_Switch = DR16_Switch_Status_UP;
    Data.Mouse_Left_Key = DR16_Key_Status_FREE;
    Data.Mouse_Right_Key = DR16_Key_Status_FREE;
    
    for(int i = 0; i < 16; i++) {
        Data.Keyboard_Key[i] = DR16_Key_Status_FREE;
    }

}

/**
 * @brief 判断拨动开关状态
 *
 */
void Class_DR16::Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    //带触发的判断
    switch (Pre_Status)
    {
    case (SWITCH_UP):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_UP_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_DOWN):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_TRIG_DOWN_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = DR16_Switch_Status_MIDDLE;
        }
        break;
        }
    }
    break;
    }
}

/**
 * @brief 判断按键状态
 *
 */
void Class_DR16::Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    //带触发的判断
    switch (Pre_Status)
    {
    case (KEY_FREE):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = DR16_Key_Status_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;
        }
        break;
        }
    }
    break;
    case (KEY_PRESSED):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = DR16_Key_Status_PRESSED;
        }
        break;
        }
    }
    break;
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_DR16::Data_Process()
{
    //数据处理过程
    Struct_DR16_UART_Data *tmp_buffer = (Struct_DR16_UART_Data *)UART_Manage_Object->Rx_Buffer;

    // 检查原始数据是否在DR16有效范围内
    if (tmp_buffer->Channel_0 < 364 || tmp_buffer->Channel_0 > 1684 ||
        tmp_buffer->Channel_1 < 364 || tmp_buffer->Channel_1 > 1684 ||
        tmp_buffer->Channel_2 < 364 || tmp_buffer->Channel_2 > 1684 ||
        tmp_buffer->Channel_3 < 364 || tmp_buffer->Channel_3 > 1684 ||
        tmp_buffer->Channel_Yaw < 364 || tmp_buffer->Channel_Yaw > 1684) {
        return; // 原始数据超出DR16有效范围
    }

    // 计算临时值
    float temp_Right_X = (tmp_buffer->Channel_0 - Rocker_Offset) / Rocker_Num;
    float temp_Right_Y = (tmp_buffer->Channel_1 - Rocker_Offset) / Rocker_Num;
    float temp_Left_X = (tmp_buffer->Channel_2 - Rocker_Offset) / Rocker_Num;
    float temp_Left_Y = (tmp_buffer->Channel_3 - Rocker_Offset) / Rocker_Num;
    float temp_Yaw = (tmp_buffer->Channel_Yaw - Rocker_Offset) / Rocker_Num;

    // 检查归一化后的值是否在有效范围内 [-1, 1]
    if (Math_Abs(temp_Right_X) > 1.0f || Math_Abs(temp_Right_Y) > 1.0f ||
        Math_Abs(temp_Left_X) > 1.0f || Math_Abs(temp_Left_Y) > 1.0f ||
        Math_Abs(temp_Yaw) > 1.0f) {
        return; // 归一化后的数据超出有效范围
    }

    // 数据有效，更新到Data结构体
    Data.Right_X = temp_Right_X;
    Data.Right_Y = temp_Right_Y;
    Data.Left_X = temp_Left_X;
    Data.Left_Y = temp_Left_Y;
    Data.Yaw = temp_Yaw;

    //判断拨码触发
    Judge_Switch(&Data.Left_Switch, tmp_buffer->Switch_1, Pre_UART_Rx_Data.Switch_1);
    Judge_Switch(&Data.Right_Switch, tmp_buffer->Switch_2, Pre_UART_Rx_Data.Switch_2);

    //鼠标信息
    Data.Mouse_X = tmp_buffer->Mouse_X / 32768.0f;
    Data.Mouse_Y = tmp_buffer->Mouse_Y / 32768.0f;
    Data.Mouse_Z = tmp_buffer->Mouse_Z / 32768.0f;

    //判断鼠标触发
    Judge_Key(&Data.Mouse_Left_Key, tmp_buffer->Mouse_Left_Key, Pre_UART_Rx_Data.Mouse_Left_Key);
    Judge_Key(&Data.Mouse_Right_Key, tmp_buffer->Mouse_Right_Key, Pre_UART_Rx_Data.Mouse_Right_Key);

    //判断键盘触发
    for (int i = 0; i < 16; i++)
    {
        Judge_Key(&Data.Keyboard_Key[i], ((tmp_buffer->Keyboard_Key) >> i) & 0x1, ((Pre_UART_Rx_Data.Keyboard_Key) >> i) & 0x1);
    }
}
/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_DR16::UART_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断遥控器是否在线
    Flag += 1;

    Data_Process();

    //保留数据
    memcpy(&Pre_UART_Rx_Data, UART_Manage_Object->Rx_Buffer, 18 * sizeof(uint8_t));
}

/**
 * @brief TIM定时器中断定期检测遥控器是否存活
 *
 */
void Class_DR16::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过遥控器数据
    if (Flag == Pre_Flag)
    {
        //遥控器断开连接
        DR16_Status = DR16_Status_DISABLE;
    }
    else
    {
        //遥控器保持连接
        DR16_Status = DR16_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/******************************************************************/
