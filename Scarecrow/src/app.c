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

void _PlaySound16bit(const uint16_t* const pData, const uint32_t nSamples);

void App_Init(void)
{
  HW_Init();

  spi_Init(pSpi, PA5, PA7, PA6);
  i2s_Init(pI2s, PB13, PB15, PB9, i2s_standard_msb, i2s_freq_8K);

}

void App_Exec(void)
{
  _PlaySound16bit(Bell_1, sizeof(Bell_1) / sizeof(uint16_t));
  Timer_Delay_ms(500);

  _PlaySound16bit(Bell_2, sizeof(Bell_2) / sizeof(uint16_t));
  Timer_Delay_ms(500);

//  uint32_t nEnd = Timer_GetTicks_ms() + 1000;
//  while (Timer_GetTicks_ms() < nEnd)
//  {
//    _PlaySound16bit(F1000, sizeof(F1000) / sizeof(uint16_t));
//  }
//
//  nEnd = Timer_GetTicks_ms() + 1000;
//  while (Timer_GetTicks_ms() < nEnd)
//  {
//    _PlaySound16bit(F500, sizeof(F500) / sizeof(uint16_t));
//  }
}

void _PlaySound16bit(const uint16_t* const pData, const uint32_t nSamples)
{
  i2s_SendMonoBuffer16(pI2s, (uint16_t*)pData, nSamples);
}
