//
// Created by luxingyu on 2023/11/17.
//

#ifndef INA226_INA226_H
#define INA226_INA226_H
#include "i2c.h"

void INA226_init(void);
//仅使用了获取电压电流功率3个功能
float INA226_GetBusV(void);
float INA226_GetCurrent(void);
float INA226_GetPower(void);

uint8_t INA226_SetShuntVoltageRegister(uint16_t ConfigWord);


#endif //INA226_INA226_H
