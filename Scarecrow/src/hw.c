/*
 * hw.c
 *
 *  Created on: 4. 12. 2019
 *  Author:     Priesol Vladimir
 */

#include "hw.h"
#include "common_f4.h"
#include "timer.h"
#include "clock_f4.h"

#include <stm32f4xx_ll_adc.h>
#include <stm32f4xx_ll_bus.h>

//#include <sys/_stdint.h>

// BLUE LED na kitu
#define BOARD_LED                PC13
#define BOARD_LED_ON             GPIO_RESETPIN(BOARD_LED)
#define BOARD_LED_OFF            GPIO_SETPIN(BOARD_LED)

#define HW_BAT                   PA0


bool HW_Init(void)
{
//  Clock_SetPLLasSysClk(8, 100, 2, CLOCK_SOURCE_HSI);  // 16 / 8 * 100 / 2 = 100Mhz

  Timer_Init();

  HW_ReadCPUID();

  // configure board LED
  GPIO_ConfigPin(BOARD_LED, mode_output, outtype_pushpull, pushpull_no, speed_low);
  BOARD_LED_OFF;

  // configure BAT analog pin
  //GPIO_ConfigPin(HW_BAT, mode_analog, outtype_pushpull, pushpull_down, speed_low);

  return true;
}

void HW_SetBoardLed(bool bOn)
{
  bOn ? BOARD_LED_ON : BOARD_LED_OFF;
}

void HW_ReadCPUID(void)
{
//  uint32_t w0 = LL_GetUID_Word0();
//  uint32_t w1 = LL_GetUID_Word1();
//  uint32_t w2 = LL_GetUID_Word2();
}
