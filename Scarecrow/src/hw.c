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
#include "watchdog.h"

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

// Voltrage measure control pin
#define HW_VOLTAGE_CTRL          PA4
#define HW_VOLTAGE_CTRL_ENABLE   GPIO_RESETPIN(HW_VOLTAGE_CTRL)
#define HW_VOLTAGE_CTRL_DISABLE  GPIO_SETPIN(HW_VOLTAGE_CTRL)

// VBAT voltage input
#define HW_VBAT                  PA5
#define HW_VBAT_ADC              LL_ADC_CHANNEL_5

// Solar panel voltage input
#define HW_SOL                   PA6
#define HW_SOL_ADC               LL_ADC_CHANNEL_6

#define APP_ADC_CONV             8       // number of conversions

uint32_t                      g_VbatVoltage_mV;
uint32_t                      g_SolVoltage_mV;

void _AdcInit(void);

bool HW_Init(void)
{
//  Clock_SetPLLasSysClk(8, 100, 2, CLOCK_SOURCE_HSI);  // 16 / 8 * 100 / 2 = 100Mhz

  // HSI 16MHz / 16 = 1MHz
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_16);

//  WDG_Init(WDG_Timeout_32s);

  Timer_Init();

  // configure board LED
  GPIO_ConfigPin(BOARD_LED, mode_output, outtype_pushpull, pushpull_no, speed_low);
  BOARD_LED_OFF;

  // supply ctrl pin and set supply off
  GPIO_ConfigPin(HW_SUPPLY_CTRL, mode_output, outtype_od, pushpull_no, speed_low);
  HW_SUPPLY_CTRL_OFF;

  // measure ctrl pin
  GPIO_ConfigPin(HW_VOLTAGE_CTRL, mode_output, outtype_od, pushpull_no, speed_low);
  HW_VOLTAGE_CTRL_DISABLE;

  GPIO_ConfigPin(HW_VBAT, mode_analog, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_SOL, mode_analog, outtype_od, pushpull_no, speed_low);

  GPIO_ConfigPin(HW_FILE_PIN0, mode_output, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN1, mode_output, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN2, mode_output, outtype_pushpull, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN3, mode_output, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN4, mode_output, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN5, mode_output, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN6, mode_output, outtype_od, pushpull_no, speed_low);
  GPIO_ConfigPin(HW_FILE_PIN7, mode_output, outtype_od, pushpull_no, speed_low);

  HW_SetFileNumber(0);

  HW_ConfigureConfigBusyPin(true);

  GPIO_ConfigPin(HW_PIR_INPUT, mode_input, outtype_pushpull, pushpull_no, speed_low);

  _AdcInit();

  return true;
}

void HW_SetMp3Supply(bool bOn)
{
  if (bOn)
  {
    HW_ConfigureConfigBusyPin(true);
    HW_SUPPLY_CTRL_ON;
    Timer_Delay_ms(300);
    HW_ConfigureConfigBusyPin(false);
  }
  else
  {
    HW_SUPPLY_CTRL_OFF;
  }
}

void HW_SetBoardLed(bool bOn)
{
  bOn ? BOARD_LED_ON : BOARD_LED_OFF;
}

void HW_VoltageMeasureControl(bool bEnable)
{
  if (bEnable == true)
  {
     HW_VOLTAGE_CTRL_ENABLE;

     // 1nF capacitor on resistor divider
     Timer_Delay_ms(2);
  }
  else
  {
    HW_VOLTAGE_CTRL_DISABLE;
  }
}

void _AdcInit(void)
{
  //enable ADC1 clock
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  // Set ADC clock (conversion clock) common to several ADC instances
  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(), LL_ADC_CLOCK_SYNC_PCLK_DIV2);

  LL_ADC_InitTypeDef ADC_Init;
  ADC_Init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_Init.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_Init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_Init);

  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);

  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
}

uint32_t HW_AdcMeasure(void)
{
  LL_ADC_Enable(ADC1);

  uint32_t nValue = 0;
  for (uint8_t i = 0; i < APP_ADC_CONV; i++)
  {
    LL_ADC_REG_StartConversionSWStart(ADC1);

    //wait for conversion complete
    while(!LL_ADC_IsActiveFlag_EOCS(ADC1));

    //read ADC value
    nValue += LL_ADC_REG_ReadConversionData12(ADC1);
  }

  LL_ADC_Disable(ADC1);

  nValue /= APP_ADC_CONV;
  return nValue;
}

void HW_MesureVoltage(void)
{
  HW_VoltageMeasureControl(true);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_ADC_SetChannelSamplingTime(ADC1, HW_VBAT_ADC, LL_ADC_SAMPLINGTIME_144CYCLES);
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);  // 1 vstup
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, HW_VBAT_ADC);
  uint32_t nValue = HW_AdcMeasure();
  nValue = nValue * 3300 / 4095;

  // resistor divider divides 2
  g_VbatVoltage_mV = nValue << 1;  // *2

  LL_ADC_SetChannelSamplingTime(ADC1, HW_SOL_ADC, LL_ADC_SAMPLINGTIME_144CYCLES);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, HW_SOL_ADC);
  nValue = HW_AdcMeasure();
  nValue = nValue * 3300 / 4095;
  g_SolVoltage_mV = nValue << 1;  // *2

  HW_VoltageMeasureControl(false);

  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
}

uint32_t HW_GetVbatVoltage_mV(void)
{
  return g_VbatVoltage_mV;
}

uint32_t HW_GetSolVoltage_mV(void)
{
  return g_SolVoltage_mV;
}

void HW_ConfigureConfigBusyPin(bool bConfigIsActive)
{
  if (bConfigIsActive == true)
  {
    GPIO_ConfigPin(HW_CONF_PIN, mode_output, outtype_pushpull, pushpull_no, speed_low);
    GPIO_RESETPIN(HW_CONF_PIN);
  }
  else
  {
    GPIO_ConfigPin(HW_CONF_PIN, mode_input, outtype_pushpull, pushpull_down, speed_low);
  }
}

bool HW_IsPlayerBusy(void)
{
  return GET_PORT(HW_BUSY_INPUT)->IDR & GET_PIN(HW_BUSY_INPUT);
}

bool HW_IsPirActive(void)
{
  return GET_PORT(HW_PIR_INPUT)->IDR & GET_PIN(HW_PIR_INPUT);
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
