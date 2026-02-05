/**
 * @file drv_can.h
 * @author wfz 
 * @version 0.0
 * @date 2026-1-4
 *
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} Struct_CAN_Rx_Buffer;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
typedef struct
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
} Struct_CAN_Manage_Object;

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0x1fe_Tx_Data[];
extern uint8_t CAN1_0x2fe_Tx_Data[];

extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0x1fe_Tx_Data[];
extern uint8_t CAN2_0x2fe_Tx_Data[];

/* Exported function declarations ---------------------------------------------*/

void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void TIM_CAN_PeriodElapsedCallback();

#endif

/*
模板：
假设你的CAN_Call_Back回调函数是
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)//你就可以用这个来处理你接收到的CAN报文了
{
    uint8_t *Rx_Data = Rx_Buffer->Data;
    switch (Rx_Buffer->Header.StdId)
    {
    case (0x201):
    {
        Rx_Encoder = (Rx_Data[0] << 8) | Rx_Data[1];
        Rx_Omega = (Rx_Data[2] << 8) | Rx_Data[3];
        Rx_Torque = (Rx_Data[4] << 8) | Rx_Data[5];
        Rx_Temperature = Rx_Data[6];
    }
    break;
    }
}

CAN_Init(&hcan1,CAN_Motor_Call_Back);

假设这是一个定时调用的函数{
	//记得把CAN1_0x1ff_Tx_Data的值改成你想发的数据
	TIM_CAN_PeriodElapsedCallback()；//你就可以定时的发送CAN报文了，假设你要发送报文为0x1ff的报文，你就要到这个函数里把该行取消注释掉。
}


*/

/******************************************************************/
