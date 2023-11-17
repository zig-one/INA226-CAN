//
// Created by luxingyu on 2023/11/17.
//

#include "usr_main.h"
#include "main.h"
#include "gpio.h"

void User_Setup() {
}
void User_Loop()
{
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    HAL_Delay(100);
}