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

#define HW_SUPPLY_CTRL           PA8
#define HW_SUPPLY_CTRL_ON        GPIO_RESETPIN(HW_SUPPLY_CTRL)
#define HW_SUPPLY_CTRL_OFF       GPIO_SETPIN(HW_SUPPLY_CTRL)

#define HW_PIR_INPUT             PA0

#define HW_BUSY_INPUT            PA1
#define HW_CONF_PIN              PA1

#define HW_FILE_PORT             GPIOB
#define HW_FILE_PIN0             PB0
#define HW_FILE_PIN1             PB1
#define HW_FILE_PIN2             PB2
#define HW_FILE_PIN3             PB3
#define HW_FILE_PIN4             PB4
#define HW_FILE_PIN5             PB5
#define HW_FILE_PIN6             PB6
#define HW_FILE_PIN7             PB7

bool HW_Init(void)
{
//  Clock_SetPLLasSysClk(8, 100, 2, CLOCK_SOURCE_HSI);  // 16 / 8 * 100 / 2 = 100Mhz

  Timer_Init();

  // configure board LED
  GPIO_ConfigPin(BOARD_LED, mode_output, outtype_pushpull, pushpull_no, speed_low);
  BOARD_LED_OFF;

  GPIO_ConfigPin(HW_FILE_PIN0, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN1, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN2, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN3, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN4, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN5, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN6, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN7, mode_output, outtype_pushpull, pushpull_no, speed_low);

  HW_SetFileNumber(0);

  GPIO_ConfigPin(HW_CONF_PIN, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_RESETPIN(HW_CONF_PIN);

  GPIO_ConfigPin(HW_SUPPLY_CTRL, mode_output, outtype_pushpull, pushpull_no, speed_low);
  HW_SetMp3Supply(false);

  HW_SetMp3Supply(true);
  Timer_Delay_ms(300);

  GPIO_ConfigPin(HW_CONF_PIN, mode_input, outtype_pushpull, pushpull_no, speed_low);

  return true;
}

void HW_SetMp3Supply(bool bOn)
{
  bOn ? HW_SUPPLY_CTRL_ON : HW_SUPPLY_CTRL_OFF;
}

void HW_SetBoardLed(bool bOn)
{
  bOn ? BOARD_LED_ON : BOARD_LED_OFF;
}

bool HW_GetBusyInput(void)
{
  return GET_PORT(HW_BUSY_INPUT)->IDR & GET_PIN(HW_BUSY_INPUT);
}

void HW_ReadCPUID(void)
{
//  uint32_t w0 = LL_GetUID_Word0();
//  uint32_t w1 = LL_GetUID_Word1();
//  uint32_t w2 = LL_GetUID_Word2();
}

void HW_SetFileNumber(uint8_t nFileNumber)
{
  LL_GPIO_WriteOutputPort(HW_FILE_PORT, nFileNumber ^ 0xFF);
}
