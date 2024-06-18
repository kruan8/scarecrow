/*
 * i2s_f4.h
 *
 *  Created on: 22. 1. 2024
 *  Author:     Priesol Vladimir
 */

#ifndef I2S_F4_H_
#define I2S_F4_H_

#include "stm32f4xx.h"
#include "stdbool.h"
#include "stddef.h"
#include "common_f4.h"
#include "stm32f4xx_ll_spi.h"

typedef enum
{
  i2s_freq_48K = 48000U,
  i2s_freq_44K = 44100U,
  i2s_freq_32K = 32000U,
  i2s_freq_22K = 22050U,
  i2s_freq_16K = 16000U,
  i2s_freq_11K = 11025U,
  i2s_freq_8K  =  8000U,
} i2s_sample_freq;

typedef enum
{
  i2s_standard_philips = LL_I2S_STANDARD_PHILIPS,
  i2s_standard_msb = LL_I2S_STANDARD_MSB,
  i2s_standard_lsb = LL_I2S_STANDARD_LSB,
  i2s_standard_pcm_short = LL_I2S_STANDARD_PCM_SHORT,
  i2s_standard_pcm_long = LL_I2S_STANDARD_PCM_LONG,
} i2s_standard;

typedef struct
{
  SPI_TypeDef* reg;
  uint8_t  nGpioAF;
  IRQn_Type irq;
} i2s_hw_t;

typedef struct
{
  i2s_hw_t const *   pHW;
  uint32_t           nBusFrequencyHz;
//  volatile uint16_t  nWriteLen;
//  volatile uint16_t  nReadLen;
//  volatile uint8_t*  pWrite;
//  volatile uint8_t*  pRead;

  volatile bool bError;

  bool bLock;      /* bus lock */
  bool bComplete;  /* completion from ISR */

} i2s_drv_t;

extern i2s_drv_t _i2s1_drv;
#define i2s1 (&_i2s1_drv)

extern i2s_drv_t _i2s2_drv;
#define i2s2 (&_i2s2_drv)

extern i2s_drv_t _i2s3_drv;
#define i2s3 (&_i2s3_drv)

#endif /* I2S_F4_H_ */


void i2s_Init(i2s_drv_t* pDrv, gpio_pins_e eClkPin, gpio_pins_e eDataPin, gpio_pins_e eWordSelectPin, i2s_standard eStandard, i2s_sample_freq eSmapleFreq);
void i2s_SendMonoBuffer16(i2s_drv_t* pDrv, uint16_t* pData, uint32_t nDataLen);
void i2s_SendStereoBuffer16(i2s_drv_t* pDrv, uint16_t* pData, uint32_t nDataLen);
void i2s_WaitForNoBusy(i2s_drv_t* pDrv);
