/*
 * SigmaStudioFW.c
 *
 *  Created on: 2017/06/01
 *      Author: shun
 */

#include "SigmaStudioFW.h"

#if 0
I2C_TransferReturn_TypeDef I2CSPM_Transfer(I2C_TypeDef *i2c, I2C_TransferSeq_TypeDef *seq)
{
  I2C_TransferReturn_TypeDef ret;
  uint32_t timeout = I2CSPM_TRANSFER_TIMEOUT;
  /* Do a polled transfer */
  ret = I2C_TransferInit(i2c, seq);
  while (ret == i2cTransferInProgress && timeout--)
  {
    ret = I2C_Transfer(i2c);
  }
  return ret;
}
#endif

uint8_t i2c_write_data[5120 + 2] = {0x00};

int32_t SIGMA_WRITE_REGISTER_BLOCK(uint8_t devAddress, uint16_t address, uint16_t length, ADI_REG_TYPE *pData)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t  i2c_read_data[2];
  //static uint8_t i2c_write_data[5120 + 2];
  //uint8_t i2c_write_data[length + 2];

  seq.addr  = devAddress;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = (address >> 8) & 0x00FF;
  i2c_write_data[1] = address & 0x00FF;

  for (uint32_t i = 0; i < length; i++)
  {
    i2c_write_data[i + 2] = pData[i];
  }
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = length + 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(I2C0, &seq);

  if (ret != i2cTransferDone)
  {
    return((uint32_t) ret);
  }

  return((uint32_t) 0);
}

int32_t SIGMA_WRITE_REGISTER_CONTROL(uint8_t devAddress, uint16_t address, uint16_t length, ADI_REG_U8 *pData)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t  i2c_read_data[2];
  static uint8_t i2c_write_data[16 + 2];
  //uint8_t i2c_write_data[length + 2];
  //uint8_t *i2c_write_data = (uint8_t *)malloc(sizeof(uint8_t) * (length + 2));

  seq.addr  = devAddress;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = (address >> 8) & 0x00FF;
  i2c_write_data[1] = address & 0x00FF;

  for (uint32_t i = 0; i < length; i++)
  {
    i2c_write_data[i + 2] = pData[i];
  }
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = length + 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(I2C0, &seq);

  if (ret != i2cTransferDone)
  {
    return((uint32_t) ret);
  }

  //free(i2c_write_data);

  return((uint32_t) 0);
}

int32_t SIGMA_SAFELOAD_WRITE_ADDR(uint8_t devAddress, uint16_t addrAddress, uint16_t address)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t  i2c_read_data[2];
  static uint8_t i2c_write_data[4];

  seq.addr  = devAddress;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = (addrAddress >> 8) & 0x00FF;
  i2c_write_data[1] = addrAddress & 0x00FF;
  i2c_write_data[2] = (address >> 8) & 0x00FF;
  i2c_write_data[3] = address & 0x00FF;

  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 4;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(I2C0, &seq);

  if (ret != i2cTransferDone)
  {
    return((uint32_t) ret);
  }

  return((uint32_t) 0);
}

int32_t SIGMA_SAFELOAD_WRITE_DATA(uint8_t devAddress, uint16_t dataAddress, uint16_t length, ADI_REG_U8 *pData)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t  i2c_read_data[2];
  static uint8_t i2c_write_data[16];

  seq.addr  = devAddress;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = (dataAddress >> 8) & 0x00FF;
  i2c_write_data[1] = dataAddress & 0x00FF;
  i2c_write_data[2] = 0x00;

  for (uint32_t i = 0; i < length; i++)
  {
    i2c_write_data[i + 3] = pData[i];
  }

  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = length + 3;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(I2C0, &seq);

  if (ret != i2cTransferDone)
  {
    return((uint32_t) ret);
  }

  return((uint32_t) 0);
}

int32_t SIGMA_SAFELOAD_WRITE_TRANSFER_BIT(uint8_t devAddress)
{
    I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t  i2c_read_data[2];
  uint8_t i2c_safeload_data[4];

  seq.addr  = devAddress;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_safeload_data[0] = 0x08;
  i2c_safeload_data[1] = 0x1C;
  i2c_safeload_data[2] = 0x00;
  i2c_safeload_data[3] = 0x3C;

  seq.buf[0].data   = i2c_safeload_data;
  seq.buf[0].len    = 4;//length + 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(I2C0, &seq);

  if (ret != i2cTransferDone)
  {
    return((uint32_t) ret);
  }

  return((uint32_t) 0);
}