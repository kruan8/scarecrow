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

#define BEEP_PAUSE_MS       20000

typedef enum
{
  APP_MODE_BEEP,
  APP_MODE_ALARM,
} app_mode_e;

static app_mode_e eMode;

i2s_drv_t* pI2s = i2s2;
spi_drv_t* pSpi = spi1;

void _PlaySound(uint8_t nFileNumber);
void _PlaySound16bit(const uint16_t* const pData, const uint32_t nSamples);
void _BeepMode(void);
void _AlarmMode(void);

void App_Init(void)
{
  HW_Init();
  eMode = APP_MODE_BEEP;
}

void App_Exec(void)
{

  switch (eMode)
  {
  case APP_MODE_BEEP:
    _BeepMode();
    break;
  case APP_MODE_ALARM:
    _AlarmMode();
    eMode = APP_MODE_BEEP;
    Timer_Delay_ms(1000);
    break;
  default:
    break;
  }

  // wait time between beep sounds
  uint32_t nNextBeepCycle_ms = Timer_GetTicks_ms() + BEEP_PAUSE_MS;
  while (Timer_GetTicks_ms() < nNextBeepCycle_ms)
  {
    __WFI();
    if (HW_IsPirActive() == true)
    {
      eMode = APP_MODE_ALARM;
      break;
    }
  }

}

void _BeepMode(void)
{
  // beep files 1-9
  const uint8_t arrBeepSounds[] =
  {
      1, 2, 3, 4, 5, 6, 7, 8, 9
  };

  static uint32_t nBeepIdx = 0;

  _PlaySound(arrBeepSounds[nBeepIdx]);
  nBeepIdx++;
  if (nBeepIdx >= sizeof(arrBeepSounds))
  {
    nBeepIdx = 0;
  }
}

void _AlarmMode(void)
{
  // alarm files 10-19
  const uint8_t arrAlarmSounds[] =
  {
      10, 11, 12, 13, 14, 15, 16, 17, 18, 19
  };
  static uint32_t nAlarmIdx = 0;

  _PlaySound(arrAlarmSounds[nAlarmIdx]);
  nAlarmIdx++;
  if (nAlarmIdx >= sizeof(arrAlarmSounds))
  {
    nAlarmIdx = 0;
  }
}

void _PlaySound(uint8_t nFileNumber)
{
  HW_SetMp3Supply(true);

  HW_SetBoardLed(true);
  Timer_Delay_ms(100);
  HW_SetBoardLed(false);

  HW_SetFileNumber(nFileNumber);
  while(!HW_IsPlayerBusy());
  HW_SetFileNumber(0);
  while(HW_IsPlayerBusy());

  HW_SetMp3Supply(false);
}
