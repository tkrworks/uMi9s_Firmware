/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#define BSP_I2CSENSOR_PERIPHERAL HAL_I2C_PORT_I2C0

#include "uartdrv.h"
#include "i2cspm.h"

#include "udelay.h"

#include "mpu9250.h"

#include "umi9s_IC_1.h"
#include "umi9s_IC_1_PARAM.h"
   
/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = 0,//SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Define receive/transmit operation queues
DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);

// Configuration for USART0, location 11
#define MY_UART                                 \
{                                               \
  USART0,                                       \
  115200,                                       \
  PORTIO_USART0_TX_LOC,                         \
  PORTIO_USART0_RX_LOC,                         \
  usartStopbits1,                               \
  usartNoParity,                                \
  usartOVS16,                                   \
  false,                                        \
  uartdrvFlowControlHwUart,                     \
  PORTIO_USART0_CTS_PORT,                       \
  PORTIO_USART0_CTS_PIN,                        \
  PORTIO_USART0_RTS_PORT,                       \
  PORTIO_USART0_RTS_PIN,                        \
  (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue, \
  (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue, \
  PORTIO_USART0_CTS_LOC,                        \
  PORTIO_USART0_RTS_LOC                         \
}

#define MAX_BUF_SIZE 64

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

UARTDRV_HandleData_t handleData;
UARTDRV_Handle_t handle = &handleData;
bool receive_flag = false;

void transmit_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data, UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferStatus;
  (void)data;
  (void)transferCount;
}

void receive_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data, UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferStatus;
  (void)data;
  //(void)transferCount;
  
  if (transferCount > 0)
  {
    receive_flag = true;
  }
}

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
  
  writeByte(I2C0, 0x68, 0x6B, 0x00);
  UDELAY_Delay(10000);
  
  writeByte(I2C0, 0x68, 0x6B, 0x01);
  
  writeByte(I2C0, 0x68, 0x6C, 0x00);
  
  writeByte(I2C0, 0x68, 0x1A, 0x03);
  writeByte(I2C0, 0x68, 0x19, 0x04);
  
  writeByte(I2C0, 0x68, 0x1B, 0x18);
  
  writeByte(I2C0, 0x68, 0x1C, 0x18);

  writeByte(I2C0, 0x68, 0x1D, 0x03);
  
  writeByte(I2C0, 0x68, 0x37, 0x22);
  writeByte(I2C0, 0x68, 0x38, 0x01);
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

/**
 * @brief  Main function
 */
void main(void)
{
  uint8_t loop_count = 0;
  uint8_t test[1];
  
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Initialize stack
  gecko_init(&config);

  GPIO_PinModeSet(gpioPortD, 11U, gpioModeWiredAndPullUp, 1);
  //GPIO_PinModeSet(gpioPortC, 9U, gpioModeWiredOr, 1);
  //GPIO_PinModeSet(gpioPortC, 10U, gpioModeWiredOr, 1);
  //GPIO_PinModeSet(gpioPortC, 11U, gpioModeWiredOr, 1);
  
  //GPIO_PinOutClear(gpioPortC, 9U);
  //GPIO_PinOutClear(gpioPortC, 10U);
  //GPIO_PinOutClear(gpioPortC, 11U);
  
  writeByte(I2C0, 0x68, 0x6B, 0x80);
  UDELAY_Delay(500000);
  //writeByte(I2C0, 0x68, 0x6B, 0x00);
  //UDELAY_Delay(500000);
    
  //writeByte(I2C0, 0x68, 0x37, 0x02);
  //writeByte(I2C0, 0x68, 0x38, 0x00);
  //UDELAY_Delay(500000);
  
#if 0// i2c line all scan
  uint8_t i2c_dev_num = 0;
  for (int i = 1; i < 127; i++)
  {
    bool flag = writeByte(I2C0, i, 0x00, 0x00);
    if (flag)
    {
      i2c_dev_num++;
    }
  }
#endif
  
  MPU9250_Init();
  
  //uint8_t wia  = readByte(I2C0, 0x0C, WHO_AM_I_AK8963);
  //uint8_t info = readByte(I2C0, 0x0C, INFO);
  AK8963_Init();
    
  UARTDRV_InitUart_t initData = MY_UART;
  UARTDRV_InitUart(handle, &initData);
  
  uint8_t buffer[64] = {0};
    
  uint8_t *rbuffer = NULL;
  UARTDRV_Count_t receiveCount = 0;
  UARTDRV_Count_t receiveRemaining = 0;
  
  default_download_IC_1();
  
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    //evt = gecko_wait_event();
    evt = gecko_peek_event();
    
    //uint8_t int_status[1] = {0};
    //readByte(I2C0, 0x68, 0x3A, int_status);
    
    uint8_t atg[14] = {0};
    
    uint8_t accel_xyz[6] = {0};
    uint16_t ax = 0;
    uint16_t ay = 0;
    uint16_t az = 0;
    
    uint8_t temp_hl[2] = {0};
    uint16_t temp = 0;
    
    uint8_t gyro_xyz[6] = {0};
    uint16_t gx = 0;
    uint16_t gy = 0;
    uint16_t gz = 0;
    
    uint8_t mag_xyz[7] = {0};
    uint16_t mx = 0;
    uint16_t my = 0;
    uint16_t mz = 0;
    
    if (readByte(I2C0, 0x68, 0x3A) & 0x01)
    {
      readBytes(I2C0, 0x68, 0x3B, 6, accel_xyz);
      ax = (uint16_t)((accel_xyz[0] << 8) | accel_xyz[1]);
      ay = (uint16_t)((accel_xyz[2] << 8) | accel_xyz[3]);
      az = (uint16_t)((accel_xyz[4] << 8) | accel_xyz[5]);
      
      readBytes(I2C0, 0x68, 0x41, 2, temp_hl);
      temp = (uint16_t)((temp_hl[0] << 8) | temp_hl[1]);
      
#if 0
      readBytes(I2C0, 0x68, 0x43, 6, gyro_xyz);
      gx = (uint16_t)((gyro_xyz[0] << 8) | gyro_xyz[1]);
      gy = (uint16_t)((gyro_xyz[2] << 8) | gyro_xyz[3]);
      gz = (uint16_t)((gyro_xyz[4] << 8) | gyro_xyz[5]);
#else
      for (int i = 0; i < 6; i++)
      {
        gyro_xyz[i] = readByte(I2C0, 0x68, 0x43 + i);
      }
      gx = (uint16_t)((gyro_xyz[0] << 8) | gyro_xyz[1]);
      gy = (uint16_t)((gyro_xyz[2] << 8) | gyro_xyz[3]);
      gz = (uint16_t)((gyro_xyz[4] << 8) | gyro_xyz[5]);
#endif
    }
        
    uint8_t mag_state = readByte(I2C0, AK8963_ADDRESS, AK8963_ST1);
    if (mag_state & 0x01)
    {
#if 0
      readBytes(I2C0, AK8963_ADDRESS, AK8963_XOUT_L, 7, mag_xyz);
#else
      for (int i = 0; i < 7; i++)
      {
        mag_xyz[i] = readByte(I2C0, AK8963_ADDRESS, AK8963_XOUT_L + i);
      }
#endif
      
      if (!(mag_xyz[6] & 0x08))
      {
        mx = (int16_t)((mag_xyz[1] << 8) | mag_xyz[0]);
        my = (int16_t)((mag_xyz[3] << 8) | mag_xyz[2]);
        mz = (int16_t)((mag_xyz[5] << 8) | mag_xyz[4]);
      }
    }
    
    if (loop_count)
    {
      GPIO_PinOutSet(gpioPortD, 11U);
      //GPIO_PinOutSet(gpioPortC, 9U);
      
      //MPU9250_Test(I2C0, 0x68, test);
      
      UARTDRV_GetReceiveStatus(handle, &rbuffer, &receiveCount, &receiveRemaining);
      UARTDRV_Receive(handle, buffer, 10, receive_callback);

      if (buffer[receiveCount - 1] == 0x0d)
      {
        buffer[receiveCount] = 0x0d;
        buffer[receiveCount + 1] = 0x0a;
        UARTDRV_Transmit(handle, buffer, receiveCount + 2, transmit_callback);
        
        UARTDRV_Abort(handle, uartdrvAbortReceive);
      }
      
#if 0
      if (receive_flag)
      {
        buffer[5] = rbuffer[0];
        UARTDRV_Transmit(handle, buffer, 8, transmit_callback);
        
        receive_flag = false;
        memset(rbuffer, 0, sizeof(rbuffer));
      }
#endif
      
      loop_count = 0;
    }
    else
    {
      GPIO_PinOutClear(gpioPortD, 11U);
      //GPIO_PinOutClear(gpioPortC, 9U);
      
      //UARTDRV_Receive(handle, rbuffer, 1, receive_callback);
      
      loop_count = 1;
    }
    
    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

        /* Set advertising parameters. 100ms advertisement interval. All channels used.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
        gecko_cmd_le_gap_set_adv_parameters(160, 160, 7);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        break;

      case gecko_evt_le_connection_closed_id:

        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Check if the user-type OTA Control Characteristic was written.
       * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:

        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
