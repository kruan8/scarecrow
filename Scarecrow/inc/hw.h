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

void HW_ReadCPUID(void);

#endif /* INC_HW_H_ */
