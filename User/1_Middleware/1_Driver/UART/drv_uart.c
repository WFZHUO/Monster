/**
 * @file drv_uart.c
 * @author WangFZhuo
 * @version 0.0
 * @date 2025-11-11
 *
 * @copyright XXU-EIStudio (c) 2025
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Struct_UART_Manage_Object UART1_Manage_Object = {0};   //丝印UART1 --> 实际USART6外设
Struct_UART_Manage_Object UART2_Manage_Object = {0};  //丝印UART2 --> 实际USART1外设
Struct_UART_Manage_Object DBUS_Manage_Object = {0};  //丝印DBUS --> 实际USART3外设

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART. 优化思路：回调函数还可优化为DMA空闲中断接收
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length)
{
    if (huart->Instance == USART6)
    {
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
        UART1_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
		HAL_UARTEx_ReceiveToIdle_IT(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART1)
    {
        UART2_Manage_Object.UART_Handler = huart;
        UART2_Manage_Object.Callback_Function = Callback_Function;
        UART2_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_IT(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        DBUS_Manage_Object.UART_Handler = huart;
        DBUS_Manage_Object.Callback_Function = Callback_Function;
        DBUS_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_IT(huart, DBUS_Manage_Object.Rx_Buffer, DBUS_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 发送数据帧. 优化思路：不再手动传入UART_HandleTypeDef *huart，传入Struct_UART_Manage_Object后自动选择UART_Handler,uint8_t *Data同理,最后只需传入要发送的字节数即可
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数,要是向定时向serialplot发送数据,就将该函数定时在TIM中断中调用. 优化思路：可更改绑定serialplot的串口，和可选择发送的通道数量和类型
 *
 */
void TIM_UART_PeriodElapsedCallback()
{
    // huart1绑定串口绘图功能,专门向serialplot发送数据或者接收serialplot的数据,且一次发送的数据为1+20*4字节,绑定serialplot为20个float类型数据的通道
    UART_Send_Data(&huart1, UART2_Manage_Object.Tx_Buffer, 1 + 20 * sizeof(float));
}

/**
 * @brief HAL库UART接收空闲中断. 优化思路：回调函数还可优化为DMA空闲中断接收
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    //选择回调函数
    if (huart->Instance == USART6)
    {
        UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, Size);
        HAL_UARTEx_ReceiveToIdle_IT(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART1)
    {
        UART2_Manage_Object.Callback_Function(UART2_Manage_Object.Rx_Buffer, Size);
        HAL_UARTEx_ReceiveToIdle_IT(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        DBUS_Manage_Object.Callback_Function(DBUS_Manage_Object.Rx_Buffer, Size);
        HAL_UARTEx_ReceiveToIdle_IT(huart, DBUS_Manage_Object.Rx_Buffer, DBUS_Manage_Object.Rx_Buffer_Length);
    }
}

/************************ XXU-EIStudio (C)**************************/
