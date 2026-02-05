/**
 * @file drv_uart.h
 * @author WangFZhuo
 * @version 0.0
 * @date 2025-11-11
 *
 * @copyright XXU-EIStudio (c) 2025
 *
 */

#ifndef DRV_UART_H
#define DRV_UART_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// Struct_UART_Manage_Object 中Tx_Buffer，Rx_Buffer的缓冲区字节长度
#define UART_BUFFER_SIZE 256

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UART_Call_Back)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief UART通信处理结构体
 */
typedef struct 
{
    UART_HandleTypeDef *UART_Handler;
    uint8_t Tx_Buffer[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer[UART_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
    UART_Call_Back Callback_Function;
}Struct_UART_Manage_Object;

/* Exported variables --------------------------------------------------------*/
//在cubeMX中初始化哪个就用取消注释哪个.注意：开发板的外壳丝印（UART1 与 UART2）与 STM32 的实际串口配置并不对应，外壳丝印UART1 对应 STM32 的 UART6，外壳丝印 UART2 对应 STM32 的 UART1。
extern UART_HandleTypeDef huart1;//对应丝印UART2
extern UART_HandleTypeDef huart3;////对应丝印DBUS
//extern UART_HandleTypeDef huart6;//对应丝印UART1

extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART2_Manage_Object;
extern Struct_UART_Manage_Object DBUS_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length);

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

void TIM_UART_PeriodElapsedCallback();

#endif

/*
使用模板：

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "drv_uart.h" 

void SystemClock_Config(void);

//为串口绘图(huart1/UART2_Manage_Object)功能绑定的回调函数,用于处理串口绘图指令
void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)  
{
    // 你处理接收数据的逻辑...
    // 例如，判断指令变量数值并执行相应操作
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  
  // 注册UART2_Manage_Object
  UART_Init(&huart1, UART_Serialplot_Call_Back, 256);  

  while (1)
  {		
     //自动发送UART2_Manage_Object.Tx_Buffer中的数据,在此程序中UART2_Manage_Object.Tx_Buffer中的数据还是初始化值，因为我们没给它赋值
		TIM_UART_PeriodElapsedCallback();  

		//若想发送任意想发的数据.例如：发送一个字节的0x55
		UART2_Manage_Object.Tx_Buffer[0] = 0x55;  
		UART_Send_Data(&huart1, UART2_Manage_Object.Tx_Buffer, 1);  

		//延时1ms
		HAL_Delay(0);
  }
  
}

*/

/************************ XXU-EIStudio (C)**************************/
