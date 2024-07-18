/*
 * watchdog.c
 *
 *  Created on: 5. 5. 2021
 *  Author:     Priesol Vladimir
 */

#include "watchdog.h"
#include "timer.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_iwdg.h"
#include "stm32f4xx_ll_system.h"

#define WDG_MAX_INTERVAL_MS             120000  // 2 minutes

static wdg_task_e         g_nTaskChecks;
static timer_tick_t       g_nLastSuccessCheck;

void WDG_Init(WDG_Timeout_e eTimeout)
{
  const uint16_t arrReloadTable[] =
  {
      5, // WDG_Timeout_5ms
     10, //WDG_Timeout_10ms,
     15, //WDG_Timeout_15ms,
     31, //WDG_Timeout_30ms,
     61, //WDG_Timeout_60ms,
    123, //WDG_Timeout_120ms
    255, //WDG_Timeout_250ms
    511, //WDG_Timeout_500ms
   1023, //WDG_Timeout_1s,
   2047, //WDG_Timeout_2s,
   4095, //WDG_Timeout_4s,
   1023, //WDG_Timeout_8s,
   2047, //WDG_Timeout_16s,
   4095, //WDG_Timeout_32s,
  };

  // Enable the peripheral clock of DBG register (uncomment for debug purpose)
  LL_DBGMCU_APB1_GRP1_FreezePeriph(LL_DBGMCU_APB1_GRP1_IWDG_STOP);

  /* Enable the peripheral clock IWDG */
  /* -------------------------------- */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1)
  {
  }

  LL_IWDG_EnableWriteAccess(IWDG);
  while (LL_IWDG_IsActiveFlag_PVU(IWDG))
  {
  }

  /* Set proper clock depending on timeout user select */
  if (eTimeout >= WDG_Timeout_8s)
  {
    /* IWDG counter clock: LSI/256 = 128Hz */
    LL_IWDG_SetPrescaler(IWDG, 0x07);
  }
  else
  {
    /* IWDG counter clock: LSI/32 = 1024Hz */
    LL_IWDG_SetPrescaler(IWDG, 0x03);
  }

  while (LL_IWDG_IsActiveFlag_RVU(IWDG))
  {
  }

  /* Set counter reload value */
  LL_IWDG_SetReloadCounter(IWDG, arrReloadTable[eTimeout]);

  LL_IWDG_ReloadCounter(IWDG);
  g_nTaskChecks = 0;
  LL_IWDG_Enable(IWDG);
}

void WDG_Check(void)
{
  if (Timer_GetTicks_ms() - g_nLastSuccessCheck > WDG_MAX_INTERVAL_MS)
  {
    if (g_nTaskChecks == wdg_all)
    {
      // Refresh the counter value with IWDG_RLR (IWDG_KR = 0x0000 AAAA)
      LL_IWDG_ReloadCounter(IWDG);
      g_nTaskChecks = 0;
      g_nLastSuccessCheck = Timer_GetTicks_ms();
    }
  }
  else
  {
    LL_IWDG_ReloadCounter(IWDG);
  }
}

void WDG_Set(wdg_task_e eTask)
{
  g_nTaskChecks |= eTask;
}

void WDG_SimpleRefresh(void)
{
  LL_IWDG_ReloadCounter(IWDG);
}
