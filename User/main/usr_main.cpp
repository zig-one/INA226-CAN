//
// Created by luxingyu on 2023/11/17.
//

#include "usr_main.h"
#include "user_can.h"
#include "stm32f1xx_hal_can.h"
#include "tim.h"
#include "main.h"
#include "gpio.h"
#include "ina226.h"
#include "cstring"
#include <string>
#include "cstdio"
#include "can.h"
#include "usart.h"
#include "cmath"
using namespace std;
#define Master_ID 0x23
#define My_ID 0x23


CAN_TxHeaderTypeDef CAN_Tx_Header;
uint8_t CAN_Tx_Buffer[8];
uint32_t send_mail_box;
float current=0;
float voltage=0;
float power=0;



/** PVD (Programmable Votage Detector) ,即可编程电压监测器，PVD中断回调，在这个函数中添加自己需要的断电时处理的内容
  * @brief  PWR PVD interrupt callback
  * @retval None
  */
void HAL_PWR_PVDCallback(void)
{
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PWR_PVDCallback could be implemented in the user file
     */
    // 断电时点亮一下LED，会亮一瞬间
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    while (1)
    {
        ;
    }
}

void sendDataByCAN() {

    CAN_Tx_Header.StdId=My_ID;
    CAN_Tx_Header.IDE=CAN_ID_STD;
    CAN_Tx_Header.RTR=CAN_RTR_DATA;
    CAN_Tx_Header.DLC=0x08;

    memcpy(CAN_Tx_Buffer,&current,4);
    memcpy(CAN_Tx_Buffer+4,&voltage,4);

    HAL_CAN_AddTxMessage(&hcan,&CAN_Tx_Header, CAN_Tx_Buffer, &send_mail_box);
}
char buf[100]={65,65,0};

void float_to_string(float f,char *str, int precision)
{
    float ff;
    int a,b,c,k,l=0,m,i=0,j;
    ff = f;

    if(f<0.0)
    {

        str[i++]='-';
        f*=-1;
    }

    a=f;
    f-=a;
    k = 0;

    while(1)
    {
        l = pow(10,k);
        m = a/l;
        if(m==0)
        {
            break;
        }
        k++;
    }
    k--;

    for(l=k+1;l>0;l--)
    {
        b = pow(10,l-1);
        c = a/b;
        str[i++]=c+48;
        a%=b;
    }
    if(precision != 0)
        str[i++] = '.';

    for(l=0;l<precision;l++)
    {
        f*=10.0;
        b = f;
        str[i++]=b+48;
        f-=b;
    }

    str[i]='\0';
}
void sendDataByUART() {

    memset(buf,0, size(buf));
    uint16_t len=1;
    buf[0]='I';
    float_to_string(current,buf+len,3);
    len= strlen(buf);
    buf[len]='U';
    len+=1;
    float_to_string(voltage,buf+len,3);
    len= strlen(buf);
    buf[len]='\n';
    len= strlen(buf);

    HAL_UART_Transmit_IT(&huart1,(uint8_t *)buf,len);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *_htim)
{
    if(_htim == &htim2)
    {
        static int LED_CNT=0;
        LED_CNT+=1;
        if(LED_CNT%500==0)
        {
            HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

        }
        if(LED_CNT>5000)
        {
//            HAL_NVIC_SystemReset();    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

            LED_CNT=0;
        }
        if(LED_CNT%2==0)
        {
            if(HAL_CAN_GetError(&hcan)!=HAL_OK)
            {
                HAL_NVIC_SystemReset();
            }
            sendDataByCAN();
            sendDataByUART();
        }

    }

}




void User_Setup() {
    INA226_init();
    CAN_Init();
    CAN_Filter_Mask_Config(CAN_FILTER(1)|CAN_FIFO_0|CAN_STDID|CAN_DATA_TYPE,0x00,0x00);
    HAL_TIM_Base_Start_IT(&htim2);

}





void User_Loop() {
    current = INA226_GetCurrent();
    voltage = INA226_GetBusV();
    power = current * voltage;
}