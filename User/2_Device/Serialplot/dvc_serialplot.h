/**
 * @file dvc_motor.h
 * @author wfz
 * @brief 串口绘图
 * @version 0.0
 * @date 2026-1-4
 *
 *
 */

#ifndef DVC_SERIALPLOT_H
#define DVC_SERIALPLOT_H

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>   // 支持可变参数列表，用于Set_Data等函数
#include <string.h>   // 提供字符串处理函数，用于指令解析与变量名比对
#include <math.h>
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

//串口绘图单条指令最大长度
#define SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH (100)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 串口绘图传输数据类型
 *
 */
enum Enum_Serialplot_Data_Type
{
    Serialplot_Data_Type_UINT8 = 0,
    Serialplot_Data_Type_UINT16,
    Serialplot_Data_Type_UINT32,
    Serialplot_Data_Type_INT8,
    Serialplot_Data_Type_INT16,
    Serialplot_Data_Type_INT32,
    Serialplot_Data_Type_FLOAT,
    Serialplot_Data_Type_DOUBLE,
};

/**
 * @brief 串口绘图工具, 最多支持12个通道
 *
 */
class Class_Serialplot
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __Serialplot_Rx_Variable_Assignment_Num = 0, char **__Serialplot_Rx_Variable_Assignment_List = NULL, Enum_Serialplot_Data_Type __Serialplot_Data_Type = Serialplot_Data_Type_FLOAT, uint8_t __Frame_Header = 0xab);

    int8_t Get_Variable_Index();
    double Get_Variable_Value();

    void Set_Data(uint8_t Number, ...);

    void UART_RxCpltCallback(uint8_t *Rx_Data);
    void TIM_Write_PeriodElapsedCallback();

protected:
    //初始化相关常量

    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;

    //预定义的串口指令字典中变量名的数量
    // 假设用户定义了3个可接收的指令：
    // - speed=100#
    // - angle=45#
    // - mode=1#
    // 那么 UART_Rx_Variable_Num 的值就是 3
    uint8_t UART_Rx_Variable_Num;
    //接收指令字典列表指针
    char **UART_Rx_Variable_List;
    //串口绘图数据类型
    Enum_Serialplot_Data_Type UART_Tx_Data_Type;
    //数据包头标
    uint8_t Frame_Header;

    //常量

    //内部变量

    //需要绘图的各个变量数据地址
    const void *Data[20];
    //当前发送的数据长度, 等价于新数据偏移量
    uint8_t Data_Number = 0;
    //当前接收的指令在指令字典中的编号
    int8_t Variable_Index = 0;
    //当前接收的指令在指令字典中的值
    float Variable_Value = 0.0f;

    //读变量

    //写变量

    //读写变量

    //内部函数

    uint8_t Judge_Variable_Name();
    void Judge_Variable_Value(int flag);
    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#endif

/*
使用模板：

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"


#include "dvc_serialplot.h"

Class_Serialplot serialplot;
static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    "po",
    "io",
    "do",
};

void SystemClock_Config(void);

void UART_Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    serialplot.UART_RxCpltCallback(Buffer);
    switch (serialplot.Get_Variable_Index())
    {
        case(0):
        {
            //处理接收到的serialplot指令po,例如设置pid_omega的比例系数
            //pid_omega.Set_K_P(serialplot.Get_Variable_Value());
        }
        break;
        case(1):
        {
            //处理接收到的serialplot指令io,例如设置pid_omega的积分系数
            //pid_omega.Set_K_I(serialplot.Get_Variable_Value());
        }
        break;
        case(2):
        {
            //处理接收到的serialplot指令do,例如设置pid_omega的微分系数
            //pid_omega.Set_K_D(serialplot.Get_Variable_Value());
        }
        break;
    }
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  
  UART_Init(&huart1, UART_Serialplot_Call_Back, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
  serialplot.Init(&huart1, 3, (char **)Variable_Assignment_List);
  
  while (1)
  {
    //举个例子：
    //假设我们要绘制一个简单的角度响应曲线，其中目标角度为5.0f，当前角度为3.0f。
    //我们可以将这两个值传递给serialplot.Set_Data(2, &Now_Omega, &Target_Omega)，
    //并在串口绘图工具中查看实时的角度响应曲线。
    Target_Omega = 5.0f;
    Now_Omega = 3.0f;
    serialplot.Set_Data(2, &Now_Omega, &Target_Omega);

    serialplot.TIM_Write_PeriodElapsedCallback();//将要向serialplot发送的数据输出到UART_Manage_Object发送缓冲区
    TIM_UART_PeriodElapsedCallback();//发送UART_Manage_Object发送缓冲区的数据到serialplot(串口)
    
    HAL_Delay(0);

  }
}


*/

/******************************************************************/
