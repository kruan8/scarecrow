/*
 * watchdog.h
 *
 *  Created on: 5. 5. 2021
 *  Author:     Priesol Vladimir
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef enum {
  WDG_Timeout_5ms =   0x00,   /*!< System reset called every 5ms */
  WDG_Timeout_10ms =  0x01,  /*!< System reset called every 10ms */
  WDG_Timeout_15ms =  0x02,  /*!< System reset called every 15ms */
  WDG_Timeout_30ms =  0x03,  /*!< System reset called every 30ms */
  WDG_Timeout_60ms =  0x04,  /*!< System reset called every 60ms */
  WDG_Timeout_120ms = 0x05, /*!< System reset called every 120ms */
  WDG_Timeout_250ms = 0x06, /*!< System reset called every 250ms */
  WDG_Timeout_500ms = 0x07, /*!< System reset called every 500ms */
  WDG_Timeout_1s =    0x08,    /*!< System reset called every 1s */
  WDG_Timeout_2s =    0x09,    /*!< System reset called every 2s */
  WDG_Timeout_4s =    0x0A,    /*!< System reset called every 4s */
  WDG_Timeout_8s =    0x0B,    /*!< System reset called every 8s */
  WDG_Timeout_16s =   0x0C,   /*!< System reset called every 16s */
  WDG_Timeout_32s =   0x0D    /*!< System reset called every 32s. This is maximum value allowed with IWDG timer */
} WDG_Timeout_e;

typedef enum
{
  wdg_main = 0b001,
  wdg_modem = 0b010,
  wdg_protocol = 0b100,
  wdg_all = 0b111
} wdg_task_e;

void WDG_Init(WDG_Timeout_e eTimeout);
void WDG_Check(void);
void WDG_Set(wdg_task_e eTask);
void WDG_SimpleRefresh(void);

#endif /* WATCHDOG_H_ */
