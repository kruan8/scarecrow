/*
 * hw.h
 *
 *  Created on: 4. 12. 2019
 *  Author:     Priesol Vladimir
 */

#ifndef INC_HW_H_
#define INC_HW_H_

#include "stm32f4xx.h"
#include <stdbool.h>

bool HW_Init(void);
void HW_SetBoardLed(bool bOn);
void HW_VoltageMeasureControl(bool bEnable);
void HW_MesureVoltage(void);
uint32_t HW_GetVbatVoltage_mV(void);
uint32_t HW_GetSolVoltage_mV(void);
void HW_SetMp3Supply(bool bOn);
bool HW_IsPlayerBusy(void);
bool HW_IsPirActive(void);
void HW_ConfigureConfigBusyPin(bool bConfig);

void HW_ReadCPUID(void);

void HW_SetFileNumber(uint8_t nFileNumber);

#endif /* INC_HW_H_ */
