//
// Created by luxingyu on 2023/11/17.
//

#include "usr_main.h"
#include "main.h"
#include "gpio.h"
#include "ina226.h"
void User_Setup() {
    INA226_init();
}
void User_Loop()
{
    float cur= INA226_GetCurrent();
    float vol= INA226_GetBusV();
    float  power= cur*vol;

    static  int t=0;
    t++;
    if(t>1000) {
        t=0;
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

    }
    HAL_Delay(1);
}