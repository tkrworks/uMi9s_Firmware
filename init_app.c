/***************************************************************************//**
 * @file init_app.c
 *******************************************************************************
 * # License
 * <b>Copyright 2017 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#include "hal-config.h"
#else
#include "bspconfig.h"
#endif

#include "i2cspm.h"
//#include "pti.h"
#include "bsp.h"

void initApp(void)
{
  // Enable PTI
  //configEnablePti();

#if 0//defined(HAL_VCOM_ENABLE)
  // Enable VCOM if requested
  GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT, BSP_VCOM_ENABLE_PIN, gpioModePushPull, HAL_VCOM_ENABLE);
#endif // HAL_VCOM_ENABLE

#if (HAL_I2CSENSOR_ENABLE)
  // Initialize I2C peripheral
  I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
  I2CSPM_Init(&i2cInit);
#endif // HAL_I2CSENSOR_ENABLE

  //test
  I2CSPM_Init_TypeDef init = I2CSPM_INIT_DEFAULT;

  init.sclPort = PORTIO_I2C0_SCL_PORT;
  init.sclPin = PORTIO_I2C0_SCL_PIN;
  init.sdaPort = PORTIO_I2C0_SDA_PORT;
  init.sdaPin = PORTIO_I2C0_SDA_PIN;
  init.portLocationScl = PORTIO_I2C0_SCL_LOC;
  init.portLocationSda = PORTIO_I2C0_SDA_LOC;
  init.i2cMaxFreq = 93458;//92000;
  //init.i2cMaxFreq = 392157;
  //init.i2cClhr = i2cClockHLRFast;
  //init.i2cClhr = i2cClockHLRAsymetric;
  
  I2CSPM_Init(&init);
  
#if 0//defined(HAL_I2CSENSOR_ENABLE)
  // Enable I2C sensor if requested
  GPIO_PinModeSet(BSP_I2CSENSOR_ENABLE_PORT, BSP_I2CSENSOR_ENABLE_PIN, gpioModePushPull, HAL_I2CSENSOR_ENABLE);
#endif // HAL_I2CSENSOR_ENABLE
}