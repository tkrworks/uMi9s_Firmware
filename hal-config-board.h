#ifndef HAL_CONFIG_BOARD_H
#define HAL_CONFIG_BOARD_H

#include "em_device.h"
#include "hal-config-types.h"

#if 0
// $[CMU]
#define BSP_CLK_HFXO_PRESENT                          (1)
#define BSP_CLK_HFXO_FREQ                             (38400000)
#define BSP_CLK_HFXO_INIT                              CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE                            (346)
#define BSP_CLK_HFXO_CTUNE_TOKEN                      (0)

#define BSP_CLK_LFXO_PRESENT                          (1)
#define BSP_CLK_LFXO_FREQ                             (32768)
#define BSP_CLK_LFXO_INIT                              CMU_LFXOINIT_DEFAULT
// [CMU]$

// $[DCDC]
#define BSP_DCDC_PRESENT                              (1)
#define BSP_DCDC_INIT                                  EMU_DCDCINIT_DEFAULT
// [DCDC]$

// $[I2C0]
#define PORTIO_I2C0_SCL_PIN              (10U)
#define PORTIO_I2C0_SCL_PORT             (gpioPortC)
#define PORTIO_I2C0_SCL_LOC              (14U)

#define PORTIO_I2C0_SDA_PIN              (11U)
#define PORTIO_I2C0_SDA_PORT             (gpioPortC)
#define PORTIO_I2C0_SDA_LOC              (16U)

#define HAL_I2C0_ENABLE                  (1)

#define BSP_I2C0_SDA_PIN                 (11U)
#define BSP_I2C0_SDA_PORT                (gpioPortC)
#define BSP_I2C0_SDA_LOC                 (16U)

#define BSP_I2C0_SCL_PIN                 (10U)
#define BSP_I2C0_SCL_PORT                (gpioPortC)
#define BSP_I2C0_SCL_LOC                 (14U)

#define HAL_I2C0_INIT_ENABLE             (1)

// [I2C0]$

// $[UARTNCP]
#define BSP_UARTNCP_USART_PORT                        (HAL_SERIAL_PORT_USART0)
#define BSP_UARTNCP_CTS_PIN                           (2)
#define BSP_UARTNCP_CTS_PORT                          (gpioPortA)
#define BSP_UARTNCP_CTS_LOC                           (30)

#define BSP_UARTNCP_RX_PIN                            (1)
#define BSP_UARTNCP_RX_PORT                           (gpioPortA)
#define BSP_UARTNCP_RX_LOC                            (0)

#define BSP_UARTNCP_TX_PIN                            (0)
#define BSP_UARTNCP_TX_PORT                           (gpioPortA)
#define BSP_UARTNCP_TX_LOC                            (0)

#define BSP_UARTNCP_RTS_PIN                           (3)
#define BSP_UARTNCP_RTS_PORT                          (gpioPortA)
#define BSP_UARTNCP_RTS_LOC                           (30)
// [UARTNCP]$
// Note: These might be necessary for the selected mcu or board as well.
// #define HAL_PA_ENABLE                                 (1)
// #define FEATURE_PA_HIGH_POWER

// $[USART0]
#define PORTIO_USART0_CTS_PIN             (8U)
#define PORTIO_USART0_CTS_PORT            (gpioPortC)
#define PORTIO_USART0_CTS_LOC             (9U)

#define PORTIO_USART0_RTS_PIN             (9U)
#define PORTIO_USART0_RTS_PORT            (gpioPortC)
#define PORTIO_USART0_RTS_LOC             (9U)

#define PORTIO_USART0_RX_PIN              (6U)
#define PORTIO_USART0_RX_PORT             (gpioPortC)
#define PORTIO_USART0_RX_LOC              (10U)

#define PORTIO_USART0_TX_PIN              (7U)
#define PORTIO_USART0_TX_PORT             (gpioPortC)
#define PORTIO_USART0_TX_LOC              (12U)

#define HAL_USART0_ENABLE                 (1)

#define BSP_USART0_CTS_PIN                (8U)
#define BSP_USART0_CTS_PORT               (gpioPortC)
#define BSP_USART0_CTS_LOC                (9U)

#define BSP_USART0_RX_PIN                 (6U)
#define BSP_USART0_RX_PORT                (gpioPortC)
#define BSP_USART0_RX_LOC                 (10U)

#define BSP_USART0_TX_PIN                 (7U)
#define BSP_USART0_TX_PORT                (gpioPortC)
#define BSP_USART0_TX_LOC                 (12U)

#define BSP_USART0_RTS_PIN                (9U)
#define BSP_USART0_RTS_PORT               (gpioPortC)
#define BSP_USART0_RTS_LOC                (9U)

#define HAL_USART0_BAUD_RATE             (115200UL)
#define HAL_USART0_FLOW_CONTROL          (HAL_USART_FLOW_CONTROL_NONE)
// [USART0]$
#else
// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[ADC0]
// [ADC0]$

// $[ANTDIV]
// [ANTDIV]$

// $[BATTERYMON]
// [BATTERYMON]$

// $[BTL_BUTTON]
// [BTL_BUTTON]$

// $[BULBPWM]
// [BULBPWM]$

// $[BULBPWM_COLOR]
// [BULBPWM_COLOR]$

// $[BUTTON]
// [BUTTON]$

// $[CMU]
#define HAL_CLK_HFCLK_SOURCE              (HAL_CLK_HFCLK_SOURCE_HFRCO)
#define HAL_CLK_LFECLK_SOURCE             (HAL_CLK_LFCLK_SOURCE_DISABLED)
#define HAL_CLK_LFBCLK_SOURCE             (HAL_CLK_LFCLK_SOURCE_DISABLED)
#define BSP_CLK_LFXO_PRESENT              (1)
#define BSP_CLK_HFXO_PRESENT              (1)
#define BSP_CLK_LFXO_INIT                  CMU_LFXOINIT_DEFAULT
#define BSP_CLK_LFXO_CTUNE                (0U)
#define BSP_CLK_LFXO_FREQ                 (32768U)
#define HAL_CLK_LFACLK_SOURCE             (HAL_CLK_LFCLK_SOURCE_DISABLED)
#define BSP_CLK_HFXO_FREQ                 (38400000UL)
#define BSP_CLK_HFXO_CTUNE                (346)
#define BSP_CLK_HFXO_INIT                  CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE_TOKEN          (0)
#define HAL_CLK_HFXO_AUTOSTART            (HAL_CLK_HFXO_AUTOSTART_NONE)
// [CMU]$

// $[COEX]
// [COEX]$

// $[CS5463]
// [CS5463]$

// $[DCDC]
#define BSP_DCDC_PRESENT                  (1)

#define HAL_DCDC_BYPASS                   (0)
#define BSP_DCDC_INIT                      EMU_DCDCINIT_DEFAULT
// [DCDC]$

// $[EMU]
// [EMU]$

// $[EXTFLASH]
// [EXTFLASH]$

// $[EZRADIOPRO]
// [EZRADIOPRO]$

// $[GPIO]
#define PORTIO_GPIO_SWV_PIN               (2U)
#define PORTIO_GPIO_SWV_PORT              (gpioPortF)
#define PORTIO_GPIO_SWV_LOC               (0U)

#define BSP_TRACE_SWO_PIN                 (2U)
#define BSP_TRACE_SWO_PORT                (gpioPortF)
#define BSP_TRACE_SWO_LOC                 (0U)
// [GPIO]$

// $[I2C0]
#define PORTIO_I2C0_SCL_PIN               (10U)
#define PORTIO_I2C0_SCL_PORT              (gpioPortC)
#define PORTIO_I2C0_SCL_LOC               (14U)

#define PORTIO_I2C0_SDA_PIN               (11U)
#define PORTIO_I2C0_SDA_PORT              (gpioPortC)
#define PORTIO_I2C0_SDA_LOC               (16U)

#define HAL_I2C0_ENABLE                   (1)

#define BSP_I2C0_SDA_PIN                  (11U)
#define BSP_I2C0_SDA_PORT                 (gpioPortC)
#define BSP_I2C0_SDA_LOC                  (16U)

#define BSP_I2C0_SCL_PIN                  (10U)
#define BSP_I2C0_SCL_PORT                 (gpioPortC)
#define BSP_I2C0_SCL_LOC                  (14U)

#define HAL_I2C0_INIT_ENABLE              (0)
// [I2C0]$

// $[I2CSENSOR]
// [I2CSENSOR]$

// $[IDAC0]
// [IDAC0]$

// $[IOEXP]
// [IOEXP]$

// $[LED]
// [LED]$

// $[LETIMER0]
// [LETIMER0]$

// $[LEUART0]
// [LEUART0]$

// $[LFXO]
// [LFXO]$

// $[LNA]
// [LNA]$

// $[PA]
// [PA]$

// $[PCNT0]
// [PCNT0]$

// $[PORTIO]
// [PORTIO]$

// $[PRS]
// [PRS]$

// $[PTI]
// [PTI]$

// $[PYD1698]
// [PYD1698]$

// $[SERIAL]
// [SERIAL]$

// $[SPIDISPLAY]
// [SPIDISPLAY]$

// $[SPINCP]
// [SPINCP]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[UARTNCP]
// [UARTNCP]$

// $[USART0]
#define PORTIO_USART0_CTS_PIN             (8U)
#define PORTIO_USART0_CTS_PORT            (gpioPortC)
#define PORTIO_USART0_CTS_LOC             (9U)

#define PORTIO_USART0_RTS_PIN             (9U)
#define PORTIO_USART0_RTS_PORT            (gpioPortC)
#define PORTIO_USART0_RTS_LOC             (9U)

#define PORTIO_USART0_RX_PIN              (6U)
#define PORTIO_USART0_RX_PORT             (gpioPortC)
#define PORTIO_USART0_RX_LOC              (10U)

#define PORTIO_USART0_TX_PIN              (7U)
#define PORTIO_USART0_TX_PORT             (gpioPortC)
#define PORTIO_USART0_TX_LOC              (12U)

#define HAL_USART0_ENABLE                 (1)

#define BSP_USART0_CTS_PIN                (8U)
#define BSP_USART0_CTS_PORT               (gpioPortC)
#define BSP_USART0_CTS_LOC                (9U)

#define BSP_USART0_RX_PIN                 (6U)
#define BSP_USART0_RX_PORT                (gpioPortC)
#define BSP_USART0_RX_LOC                 (10U)

#define BSP_USART0_TX_PIN                 (7U)
#define BSP_USART0_TX_PORT                (gpioPortC)
#define BSP_USART0_TX_LOC                 (12U)

#define BSP_USART0_RTS_PIN                (9U)
#define BSP_USART0_RTS_PORT               (gpioPortC)
#define BSP_USART0_RTS_LOC                (9U)

#define HAL_USART0_BAUD_RATE              (115200UL)
#define HAL_USART0_FLOW_CONTROL           (HAL_USART_FLOW_CONTROL_NONE)
// [USART0]$

// $[USART1]
// [USART1]$

// $[VCOM]
// [VCOM]$

// $[VUART]
// [VUART]$

// $[WDOG]
// [WDOG]$
#endif

#endif /* HAL_CONFIG_BOARD_H */
