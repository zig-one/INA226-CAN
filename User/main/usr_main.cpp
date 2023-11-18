//
// Created by luxingyu on 2023/11/17.
//

#include "usr_main.h"
#include "user_can.h"
#include "tim.h"
#include "main.h"
#include "gpio.h"
#include "ina226.h"
#include "cstring"
#include <string>
#include "cstdio"
#include "can.h"
#include "usart.h"
using namespace std;
#define Master_ID 0
#define My_ID 27


CAN_TxHeaderTypeDef CAN_Tx_Header;
uint8_t CAN_Tx_Buffer[8];
uint32_t send_mail_box;
float current=0;
float voltage=0;
float power=0;


void sendDataByCAN() {

    CAN_Tx_Header.StdId=My_ID;
    CAN_Tx_Header.IDE=CAN_ID_STD;
    CAN_Tx_Header.RTR=CAN_RTR_DATA;
    CAN_Tx_Header.DLC=0x08;

    memcpy(CAN_Tx_Buffer,&current,4);
    memcpy(CAN_Tx_Buffer+4,&voltage,4);


    HAL_CAN_AddTxMessage(&hcan,&CAN_Tx_Header, CAN_Tx_Buffer, &send_mail_box);
}

void sendDataByUART() {
    char buf[50]={0};
    int len=sprintf(buf,"%f,%f",current,voltage);
    HAL_UART_Transmit(&huart1,(uint8_t *)buf,len,3);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *_htim)
{
    if(_htim == &htim2)
    {
        static int LED_CNT=0;
        LED_CNT+=1;
        if(LED_CNT>500)
        {
            HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
            LED_CNT=0;
        }
        if(LED_CNT%2==0)
        {
            sendDataByCAN();
        }

    }

}




void User_Setup() {
    INA226_init();
    CAN_Init();
    CAN_Filter_Mask_Config(CAN_FILTER(1)|CAN_FIFO_0|CAN_STDID|CAN_DATA_TYPE,0x00,0x00);
}





void User_Loop() {
    current = INA226_GetCurrent();
    voltage = INA226_GetBusV();
    power = current * voltage;
}