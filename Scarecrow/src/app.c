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
#include "watchdog.h"

#define BEEP_PAUSE_MS       10000

i2s_drv_t* pI2s = i2s2;
spi_drv_t* pSpi = spi1;

void _PlaySound(uint8_t nFileNumber);
void _PlaySound16bit(const uint16_t* const pData, const uint32_t nSamples);

void App_Init(void)
{
  HW_Init();

}

void App_Exec(void)
{
  // beep files 1-7
  // alarm files 10-18

  const uint8_t arrBeepSounds[] =
  {
      1, 2, 3, 4, 5, 6, 7, 4, 2, 1, 7, 3, 6, 5
  };

  const uint8_t arrAlarmSounds[] =
  {
      10, 11, 12, 13, 14, 15, 16, 17, 18,
  };

  uint8_t nAlarmIdx = 0;

  for (uint8_t i = 0; i < sizeof (arrBeepSounds); i++)
  {
    _PlaySound(arrBeepSounds[i]);

    // wait time between beep sounds
    uint32_t nEndBeepTime = Timer_GetTicks_ms() + BEEP_PAUSE_MS;
    while (Timer_GetTicks_ms() < nEndBeepTime)
    {

      __WFI();

      WDG_SimpleRefresh();

      if (HW_IsPirActive() == true)
      {
        nEndBeepTime = Timer_GetTicks_ms();
      }
    }

    // loop for alarm sound
    while (HW_IsPirActive() == true)
    {
      _PlaySound(arrAlarmSounds[nAlarmIdx]);
      Timer_Delay_ms(1000);
      nAlarmIdx++;
      if (nAlarmIdx >= sizeof(arrAlarmSounds))
      {
        nAlarmIdx = 0;
      }
    }
  }

}

void _PlaySound(uint8_t nFileNumber)
{
  HW_SetMp3Supply(true);

  HW_SetFileNumber(nFileNumber);
  while(!HW_IsPlayerBusy());
  HW_SetFileNumber(0);
  while(HW_IsPlayerBusy());

  HW_SetMp3Supply(false);

  HW_SetBoardLed(true);
  Timer_Delay_ms(100);
  HW_SetBoardLed(false);

}
