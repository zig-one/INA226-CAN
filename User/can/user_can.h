/**
 * @file drv_can.h
 * @author 地表最强恐龙
 * @brief 一定包含Queue_CAN相关函数
 * @version 0.1
 * @date 2022-11-3
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
#include "can.h"
/* Exported macros -----------------------------------------------------------*/

#define CAN_FILTER(x) ((x) << 3)

#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

#define QUEUESIZE_CAN   50
#define TICK_TIME_OUT   1000

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN发送的信息结构体
 *
 */
typedef struct Struct_CAN_Tx_Buffer
{
    CAN_HandleTypeDef *hcan;
    uint16_t ID;
    uint32_t ID_Ext;
    uint8_t Data[16];
    uint16_t Length;
    uint16_t Tick_Wait;
}Struct_CAN_Tx_Buffer;


/**
 * @brief CAN接收的信息结构体
 *
 */
typedef struct Struct_CAN_Rx_Buffer
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
}Struct_CAN_Rx_Buffer;


/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct Struct_CAN_Manage_Object
{
    CAN_HandleTypeDef *CAN_Handler;
    CAN_Call_Back Callback_Function;
};

/* Exported variables ---------------------------------------------------------*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

extern Struct_CAN_Tx_Buffer CAN_Tx_Buffer_1;
extern Struct_CAN_Tx_Buffer CAN_Tx_Buffer_2;

extern Struct_CAN_Rx_Buffer CAN_RxBuffer_1;
extern Struct_CAN_Rx_Buffer CAN_RxBuffer_2;

/* Exported function declarations ---------------------------------------------*/

void CAN_Init();

void CAN_Filter_Mask_Config(uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

uint8_t CAN_Send_Data( uint16_t ID, uint8_t *Data, uint16_t Length);

uint8_t CAN_Send_Data_ExtID(uint32_t ID, uint8_t *Data, uint16_t Length);



void My_CAN_RxCallback(Struct_CAN_Rx_Buffer* buffer);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
