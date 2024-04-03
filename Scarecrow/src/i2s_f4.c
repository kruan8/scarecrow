/*
 * i2s_f4.c
 *
 *  Created on: 22. 1. 2024
 *  Author:     Priesol Vladimir
 */


#include "i2s_f4.h"
#include "clock_f4.h"

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"

const i2s_hw_t i2s1_hw =
{
  .reg = SPI1,
  .nGpioAF = LL_GPIO_AF_5,
  .irq = SPI1_IRQn,
};

const i2s_hw_t i2s2_hw =
{
  .reg = SPI2,
  .nGpioAF = LL_GPIO_AF_5,
  .irq = SPI2_IRQn,
};

const i2s_hw_t i2s3_hw =
{
  .reg = SPI3,
  .nGpioAF = LL_GPIO_AF_6,
  .irq = SPI3_IRQn,
};

i2s_drv_t _i2s1_drv = { &i2s1_hw, 0, false, false, false };
i2s_drv_t _i2s2_drv = { &i2s2_hw, 0, false, false, false };
i2s_drv_t _i2s3_drv = { &i2s3_hw, 0, false, false, false };


void i2s_Init(i2s_drv_t* pDrv, gpio_pins_e eClkPin, gpio_pins_e eDataPin, gpio_pins_e eWordSelectPin, i2s_standard eStandard, i2s_sample_freq eSmapleFreq)
{
  GPIO_ConfigPin(eClkPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eClkPin, pDrv->pHW->nGpioAF);

  GPIO_ConfigPin(eDataPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eDataPin, pDrv->pHW->nGpioAF);

  GPIO_ConfigPin(eWordSelectPin, mode_alternate, outtype_pushpull, pushpull_no, speed_high);
  GPIO_SetAFpin(eWordSelectPin, pDrv->pHW->nGpioAF);

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks); // Get system clocks

  if (pDrv->pHW->reg == SPI1)
  {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    pDrv->nBusFrequencyHz = RCC_Clocks.PCLK2_Frequency;
  }

  if (pDrv->pHW->reg == SPI2)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    pDrv->nBusFrequencyHz = RCC_Clocks.PCLK1_Frequency;
  }
  else if (pDrv->pHW->reg == SPI3)
  {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
    pDrv->nBusFrequencyHz = RCC_Clocks.PCLK1_Frequency;
  }

  Clock_SetPLL_I2S(16, 200, 2);

  /* I2S configuration -------------------------------------------------------*/
  LL_I2S_DeInit(pDrv->pHW->reg);

  LL_I2S_InitTypeDef  I2S_InitStructure;
  I2S_InitStructure.Mode = LL_I2S_MODE_MASTER_TX;
  I2S_InitStructure.Standard = eStandard;
  I2S_InitStructure.DataFormat = LL_I2S_DATAFORMAT_16B;
  I2S_InitStructure.MCLKOutput = LL_I2S_MCLK_OUTPUT_DISABLE;
  I2S_InitStructure.AudioFreq = eSmapleFreq;
  I2S_InitStructure.ClockPolarity = LL_I2S_POLARITY_LOW;
  LL_I2S_Init(pDrv->pHW->reg, &I2S_InitStructure);

  LL_I2S_Enable(pDrv->pHW->reg);
}

void i2s_SendMonoBuffer16(i2s_drv_t* pDrv, uint16_t* pData, uint16_t nDataLen)
{
  while (nDataLen-- > 0)
  {
    while (!LL_I2S_IsActiveFlag_TXE(pDrv->pHW->reg));
    LL_SPI_TransmitData16(pDrv->pHW->reg, *pData);
    while (!LL_I2S_IsActiveFlag_TXE(pDrv->pHW->reg));
    LL_SPI_TransmitData16(pDrv->pHW->reg, *pData++);
  }
}

void i2s_SendStereoBuffer16(i2s_drv_t* pDrv, uint16_t* pData, uint16_t nDataLen)
{
  while (nDataLen-- > 0)
  {
    while (!LL_I2S_IsActiveFlag_TXE(pDrv->pHW->reg));
    LL_SPI_TransmitData16(pDrv->pHW->reg, *pData++);
  }
}

void i2s_WaitForNoBusy(i2s_drv_t* pDrv)
{
  while (LL_I2S_IsActiveFlag_BSY(pDrv->pHW->reg));
}
