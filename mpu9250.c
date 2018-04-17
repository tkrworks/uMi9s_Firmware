#include "mpu9250.h"

bool writeByte(I2C_TypeDef *i2c, uint8_t addr1, uint8_t addr2, uint8_t data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  uint8_t i2c_write_data[2];
  uint8_t i2c_read_data[1];

  seq.addr  = addr1 << 1;
  seq.flags = I2C_FLAG_WRITE;

  /* Select command to issue */
  i2c_write_data[0] = addr2;
  i2c_write_data[1] = data;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
  {
    return false;
  }
  return true;
}

uint8_t readByte0(I2C_TypeDef *i2c, uint8_t addr1)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  uint8_t i2c_write_data[1];
  uint8_t i2c_read_data[8] = {0x00};

  seq.addr  = addr1 << 1;
  seq.flags = I2C_FLAG_WRITE_READ;

  /* Select command to issue */
  //i2c_write_data[0] = addr2;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 0;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
  {
    return NULL;
  }
  
  return i2c_read_data[0];
}

uint8_t readByte(I2C_TypeDef *i2c, uint8_t addr1, uint8_t addr2)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  uint8_t i2c_write_data[1];
  uint8_t i2c_read_data[8] = {0x00};

  seq.addr  = addr1 << 1;
  seq.flags = I2C_FLAG_WRITE_READ;

  /* Select command to issue */
  i2c_write_data[0] = addr2;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
  {
    return NULL;
  }
  
  return i2c_read_data[0];
}

bool readBytes(I2C_TypeDef *i2c, uint8_t addr1, uint8_t addr2, uint8_t len, uint8_t *data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;

  uint8_t i2c_write_data[8] = {0x00};
  uint8_t i2c_read_data[8] = {0x00};

  seq.addr  = addr1 << 1;
  seq.flags = I2C_FLAG_WRITE_READ;

  /* Select command to issue */
  for (int i = 0; i < len; i++)
  {
    i2c_write_data[i] = addr2 + i;
  }
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = len;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = len;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
  {
    return false;
  }
  
  if (NULL != data)
  {
    for (int i = 0; i < len; i++)
    {
      data[i]  = i2c_read_data[i];
    }
  }
  
  return true;
}

void MPU9250_Init(void)
{
#if 0
  I2CSPM_Init_TypeDef init = I2CSPM_INIT_DEFAULT;

  init.sclPort = PORTIO_I2C0_SCL_PORT;
  init.sclPin = PORTIO_I2C0_SCL_PIN;
  init.sdaPort = PORTIO_I2C0_SDA_PORT;
  init.sdaPin = PORTIO_I2C0_SDA_PIN;
  init.portLocationScl = PORTIO_I2C0_SCL_LOC;
  init.portLocationSda = PORTIO_I2C0_SDA_LOC;
  
  I2CSPM_Init(&init);
#endif
  
  writeByte(I2C0, MPU9250_ADDRESS, PWR_MGMT_1,    0x00);
  UDELAY_Delay(10000);
  
  writeByte(I2C0, MPU9250_ADDRESS, PWR_MGMT_1,    0x01);
  
  writeByte(I2C0, MPU9250_ADDRESS, PWR_MGMT_2,    0x00);
  
  writeByte(I2C0, MPU9250_ADDRESS, CONFIG,        0x03);
  writeByte(I2C0, MPU9250_ADDRESS, SMPLRT_DIV,    0x04);
  
  writeByte(I2C0, MPU9250_ADDRESS, GYRO_CONFIG,   0x18);
  
  writeByte(I2C0, MPU9250_ADDRESS, ACCEL_CONFIG,  0x18);

  writeByte(I2C0, MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);
  
  writeByte(I2C0, MPU9250_ADDRESS, INT_PIN_CFG,   0x22);
  writeByte(I2C0, MPU9250_ADDRESS, INT_ENABLE,    0x01);
  UDELAY_Delay(10000);
}

void AK8963_Init(void)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here

  writeByte(I2C0, AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  UDELAY_Delay(100000);

  writeByte(I2C0, AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  UDELAY_Delay(100000);
  
  readBytes(I2C0, AK8963_ADDRESS, AK8963_ASAX, 3, rawData);  // Read the x-, y-, and z-axis calibration values
  float mx, my, mz;
  mx =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  my =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  mz =  (float)(rawData[2] - 128)/256.0f + 1.0f; 

  writeByte(I2C0, AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  UDELAY_Delay(10000);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(I2C0, AK8963_ADDRESS, AK8963_CNTL, 0x16); // Set magnetometer data resolution and sample ODR
  UDELAY_Delay(10000);
}
