/*
 * app.c
 *
 *  Created on: 15. 2. 2024
 *  Author:     Priesol Vladimir
 */

#include "spi_f4.h"
#include "i2s_f4.h"
#include "hw.h"
#include "sounds.inc"
#include "timer.h"

i2s_drv_t* pI2s = i2s2;
spi_drv_t* pSpi = spi1;

void _PlayBeepSound(uint8_t nFileNumber);
void _PlaySound16bit(const uint16_t* const pData, const uint32_t nSamples);

void App_Init(void)
{
  HW_Init();

//  spi_Init(pSpi, PA5, PA7, PA6);
//  i2s_Init(pI2s, PB13, PB15, PB9, i2s_standard_msb, i2s_freq_8K);

  HW_SetMp3Supply(true);

  Timer_Delay_ms(300);
}

void App_Exec(void)
{

  // beep files 1-7
  // alarm files 10-18
  _PlayBeepSound(1);
  _PlayBeepSound(2);
  _PlayBeepSound(3);
  _PlayBeepSound(4);
  _PlayBeepSound(5);
  _PlayBeepSound(6);
  _PlayBeepSound(7);
}

void _PlaySound16bit(const uint16_t* const pData, const uint32_t nSamples)
{
  i2s_SendMonoBuffer16(pI2s, (uint16_t*)pData, nSamples);
}

void _PlayBeepSound(uint8_t nFileNumber)
{
  HW_SetFileNumber(nFileNumber);
  while(!HW_GetBusyInput());

  while(HW_GetBusyInput());
  HW_SetFileNumber(0);
  Timer_Delay_ms(5000);
}
