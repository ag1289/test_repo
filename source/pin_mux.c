/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MK22FN128xxx10
package_id: MK22FN128VLH10
mcu_data: ksdk2_0
processor_version: 2.0.0
pin_labels:
- {pin_num: '1', pin_signal: ADC1_SE4a/PTE0/CLKOUT32K/SPI1_PCS1/UART1_TX/I2C1_SDA/RTC_CLKOUT, label: E_RS485_TX}
- {pin_num: '2', pin_signal: ADC1_SE5a/PTE1/LLWU_P0/SPI1_SOUT/UART1_RX/I2C1_SCL/SPI1_SIN, label: E_RS485_RX}
- {pin_num: '9', pin_signal: ADC0_DP0/ADC1_DP3, label: I_PV}
- {pin_num: '11', pin_signal: ADC1_DP0/ADC0_DP3, label: I_BATT}
- {pin_num: '17', pin_signal: VREF_OUT/CMP1_IN5/CMP0_IN5/ADC1_SE18, label: MCU_VREFOUT}
- {pin_num: '18', pin_signal: DAC0_OUT/CMP1_IN3/ADC0_SE23, label: MCU_DAC}
- {pin_num: '20', pin_signal: EXTAL32, label: RTC_CLKOUT}
- {pin_num: '26', pin_signal: PTA4/LLWU_P3/FTM0_CH1/NMI_b/EZP_CS_b, label: PWM_PV_EN, identifier: PWM_PV_EN}
- {pin_num: '27', pin_signal: PTA5/USB_CLKIN/FTM0_CH2/I2S0_TX_BCLK/JTAG_TRST_b, label: PWM_PV, identifier: PWM_PV}
- {pin_num: '28', pin_signal: PTA12/FTM1_CH0/I2S0_TXD0/FTM1_QD_PHA, label: IN_BRIDGE_1, identifier: IN_BRIDGE_1}
- {pin_num: '29', pin_signal: PTA13/LLWU_P4/FTM1_CH1/I2S0_TX_FS/FTM1_QD_PHB, label: IN_BRIDGE_2, identifier: IN_BRIDGE_2}
- {pin_num: '32', pin_signal: EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0, label: INH_BRIDGE1, identifier: INH_BRIDGE1}
- {pin_num: '33', pin_signal: XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1, label: AXIS_OUT1, identifier: AXIS_OUT1}
- {pin_num: '35', pin_signal: ADC0_SE8/ADC1_SE8/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/FTM1_QD_PHA, label: V_PV}
- {pin_num: '36', pin_signal: ADC0_SE9/ADC1_SE9/PTB1/I2C0_SDA/FTM1_CH1/FTM1_QD_PHB, label: V_BATT}
- {pin_num: '37', pin_signal: ADC0_SE12/PTB2/I2C0_SCL/UART0_RTS_b/FTM0_FLT3, label: V_REF}
- {pin_num: '38', pin_signal: ADC0_SE13/PTB3/I2C0_SDA/UART0_CTS_b/FTM0_FLT0, label: TEMP_PCB}
- {pin_num: '39', pin_signal: PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/EWM_IN, label: KEYB0, identifier: KEYB0}
- {pin_num: '40', pin_signal: PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/EWM_OUT_b, label: KEYB1, identifier: KEYB1}
- {pin_num: '41', pin_signal: PTB18/FTM2_CH0/I2S0_TX_BCLK/FTM2_QD_PHA, label: KEYB2, identifier: KEYB2}
- {pin_num: '42', pin_signal: PTB19/FTM2_CH1/I2S0_TX_FS/FTM2_QD_PHB, label: KEYB3, identifier: KEYB3}
- {pin_num: '43', pin_signal: ADC0_SE14/PTC0/SPI0_PCS4/PDB0_EXTRG/USB_SOF_OUT, label: TEMP_BATT}
- {pin_num: '44', pin_signal: ADC0_SE15/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/I2S0_TXD0/LPUART0_RTS_b, label: E_RS485_RW}
- {pin_num: '45', pin_signal: ADC0_SE4b/CMP1_IN0/PTC2/SPI0_PCS2/UART1_CTS_b/FTM0_CH1/I2S0_TX_FS/LPUART0_CTS_b, label: RTC_RST, identifier: RTC_RST}
- {pin_num: '46', pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK/LPUART0_RX, label: RTC_INT, identifier: RTC_INT}
- {pin_num: '49', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/CMP1_OUT/LPUART0_TX, label: RTC_PFO, identifier: RTC_PFO}
- {pin_num: '50', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/CMP0_OUT/FTM0_CH2, label: MEMS_INT, identifier: MEMS_INT}
- {pin_num: '51', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/I2S0_MCLK, label: RFM_RST, identifier: RFM_RST}
- {pin_num: '52', pin_signal: CMP0_IN1/PTC7/SPI0_SIN/USB_SOF_OUT/I2S0_RX_FS, label: AXIS_OUT2, identifier: AXIS_OUT2}
- {pin_num: '53', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/I2S0_MCLK, label: IS_BRIDGE1}
- {pin_num: '54', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/I2S0_RX_BCLK/FTM2_FLT0, label: IS_BRIDGE2}
- {pin_num: '55', pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/I2S0_RX_FS, label: I2C1_SCL}
- {pin_num: '56', pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA, label: I2C1_SDA}
- {pin_num: '57', pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/LPUART0_RTS_b, label: SPI0_CS0}
- {pin_num: '58', pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/LPUART0_CTS_b, label: SPI0_SCK}
- {pin_num: '59', pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/LPUART0_RX/I2C0_SCL, label: SPI0_MOSI}
- {pin_num: '60', pin_signal: PTD3/SPI0_SIN/UART2_TX/LPUART0_TX/I2C0_SDA, label: SPI0_MISO}
- {pin_num: '61', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/EWM_IN/SPI1_PCS0, label: KEYB4, identifier: KEYB4}
- {pin_num: '62', pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/FTM0_CH5/EWM_OUT_b/SPI1_SCK, label: KEYB5, identifier: KEYB5}
- {pin_num: '63', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FTM0_FLT0/SPI1_SOUT, label: KEYB6, identifier: KEYB6}
- {pin_num: '64', pin_signal: PTD7/UART0_TX/FTM0_CH7/FTM0_FLT1/SPI1_SIN, label: KEYB7, identifier: KEYB7}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*FUNCTION**********************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 *END**************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

#define PCR_PE_ENABLED                0x01u   /*!< Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
#define PCR_PS_DOWN                   0x00u   /*!< Pull Select: Internal pulldown resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
#define PCR_PS_UP                     0x01u   /*!< Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
#define PIN0_IDX                         0u   /*!< Pin number for pin 0 in a port */
#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define PIN8_IDX                         8u   /*!< Pin number for pin 8 in a port */
#define PIN9_IDX                         9u   /*!< Pin number for pin 9 in a port */
#define PIN10_IDX                       10u   /*!< Pin number for pin 10 in a port */
#define PIN11_IDX                       11u   /*!< Pin number for pin 11 in a port */
#define PIN12_IDX                       12u   /*!< Pin number for pin 12 in a port */
#define PIN13_IDX                       13u   /*!< Pin number for pin 13 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PIN18_IDX                       18u   /*!< Pin number for pin 18 in a port */
#define PIN19_IDX                       19u   /*!< Pin number for pin 19 in a port */
#define SOPT4_FTM1CH0SRC_FTM          0x00u   /*!< FTM1 channel 0 input capture source select: FTM1_CH0 signal */
#define SOPT5_UART1TXSRC_UART_TX      0x00u   /*!< UART 1 transmit data source select: UART1_TX pin */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '1', peripheral: UART1, signal: TX, pin_signal: ADC1_SE4a/PTE0/CLKOUT32K/SPI1_PCS1/UART1_TX/I2C1_SDA/RTC_CLKOUT}
  - {pin_num: '2', peripheral: UART1, signal: RX, pin_signal: ADC1_SE5a/PTE1/LLWU_P0/SPI1_SOUT/UART1_RX/I2C1_SCL/SPI1_SIN}
  - {pin_num: '5', peripheral: USB0, signal: DP, pin_signal: USB0_DP}
  - {pin_num: '6', peripheral: USB0, signal: DM, pin_signal: USB0_DM}
  - {pin_num: '7', peripheral: SUPPLY, signal: 'USBVDD, 0', pin_signal: USBVDD}
  - {pin_num: '9', peripheral: ADC0, signal: 'SE, 0', pin_signal: ADC0_DP0/ADC1_DP3}
  - {pin_num: '11', peripheral: ADC0, signal: 'SE, 3', pin_signal: ADC1_DP0/ADC0_DP3}
  - {pin_num: '17', peripheral: VREF, signal: OUT, pin_signal: VREF_OUT/CMP1_IN5/CMP0_IN5/ADC1_SE18}
  - {pin_num: '18', peripheral: DAC0, signal: OUT, pin_signal: DAC0_OUT/CMP1_IN3/ADC0_SE23}
  - {pin_num: '20', peripheral: RTC, signal: EXTAL32, pin_signal: EXTAL32}
  - {pin_num: '26', peripheral: GPIOA, signal: 'GPIO, 4', pin_signal: PTA4/LLWU_P3/FTM0_CH1/NMI_b/EZP_CS_b, direction: OUTPUT, pull_select: down}
  - {pin_num: '27', peripheral: FTM0, signal: 'CH, 2', pin_signal: PTA5/USB_CLKIN/FTM0_CH2/I2S0_TX_BCLK/JTAG_TRST_b, direction: OUTPUT, pull_select: down, pull_enable: enable}
  - {pin_num: '28', peripheral: FTM1, signal: 'CH, 0', pin_signal: PTA12/FTM1_CH0/I2S0_TXD0/FTM1_QD_PHA, direction: OUTPUT, pull_enable: enable}
  - {pin_num: '29', peripheral: FTM1, signal: 'CH, 1', pin_signal: PTA13/LLWU_P4/FTM1_CH1/I2S0_TX_FS/FTM1_QD_PHB, direction: OUTPUT, pull_enable: enable}
  - {pin_num: '32', peripheral: GPIOA, signal: 'GPIO, 18', pin_signal: EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0, direction: INPUT, pull_enable: enable}
  - {pin_num: '33', peripheral: GPIOA, signal: 'GPIO, 19', pin_signal: XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1, direction: OUTPUT, pull_enable: enable}
  - {pin_num: '35', peripheral: ADC1, signal: 'SE, 8', pin_signal: ADC0_SE8/ADC1_SE8/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/FTM1_QD_PHA, slew_rate: no_init, drive_strength: no_init,
    pull_select: no_init}
  - {pin_num: '36', peripheral: ADC1, signal: 'SE, 9', pin_signal: ADC0_SE9/ADC1_SE9/PTB1/I2C0_SDA/FTM1_CH1/FTM1_QD_PHB, slew_rate: no_init, drive_strength: no_init,
    pull_select: no_init}
  - {pin_num: '37', peripheral: ADC0, signal: 'SE, 12', pin_signal: ADC0_SE12/PTB2/I2C0_SCL/UART0_RTS_b/FTM0_FLT3, slew_rate: no_init, pull_select: no_init}
  - {pin_num: '38', peripheral: ADC0, signal: 'SE, 13', pin_signal: ADC0_SE13/PTB3/I2C0_SDA/UART0_CTS_b/FTM0_FLT0, slew_rate: no_init, pull_select: no_init}
  - {pin_num: '39', peripheral: GPIOB, signal: 'GPIO, 16', pin_signal: PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/EWM_IN, direction: INPUT, pull_select: up, pull_enable: enable}
  - {pin_num: '40', peripheral: GPIOB, signal: 'GPIO, 17', pin_signal: PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/EWM_OUT_b, direction: INPUT, pull_select: up, pull_enable: enable}
  - {pin_num: '41', peripheral: GPIOB, signal: 'GPIO, 18', pin_signal: PTB18/FTM2_CH0/I2S0_TX_BCLK/FTM2_QD_PHA, direction: INPUT, pull_select: up, pull_enable: enable}
  - {pin_num: '42', peripheral: GPIOB, signal: 'GPIO, 19', pin_signal: PTB19/FTM2_CH1/I2S0_TX_FS/FTM2_QD_PHB, direction: INPUT, pull_select: up, pull_enable: enable}
  - {pin_num: '43', peripheral: ADC0, signal: 'SE, 14', pin_signal: ADC0_SE14/PTC0/SPI0_PCS4/PDB0_EXTRG/USB_SOF_OUT}
  - {pin_num: '44', peripheral: UART1, signal: RTS, pin_signal: ADC0_SE15/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/I2S0_TXD0/LPUART0_RTS_b}
  - {pin_num: '45', peripheral: GPIOC, signal: 'GPIO, 2', pin_signal: ADC0_SE4b/CMP1_IN0/PTC2/SPI0_PCS2/UART1_CTS_b/FTM0_CH1/I2S0_TX_FS/LPUART0_CTS_b, direction: OUTPUT,
    pull_select: up, pull_enable: enable}
  - {pin_num: '46', peripheral: GPIOC, signal: 'GPIO, 3', pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK/LPUART0_RX, direction: INPUT,
    pull_select: up, pull_enable: enable}
  - {pin_num: '49', peripheral: GPIOC, signal: 'GPIO, 4', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/CMP1_OUT/LPUART0_TX, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '50', peripheral: GPIOC, signal: 'GPIO, 5', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/CMP0_OUT/FTM0_CH2, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '51', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/I2S0_MCLK, direction: OUTPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '52', peripheral: GPIOC, signal: 'GPIO, 7', pin_signal: CMP0_IN1/PTC7/SPI0_SIN/USB_SOF_OUT/I2S0_RX_FS, direction: OUTPUT, pull_enable: enable}
  - {pin_num: '53', peripheral: ADC1, signal: 'SE, 4b', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/I2S0_MCLK}
  - {pin_num: '53', peripheral: CMP0, signal: 'IN, 2', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/I2S0_MCLK}
  - {pin_num: '54', peripheral: ADC1, signal: 'SE, 5b', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/I2S0_RX_BCLK/FTM2_FLT0}
  - {pin_num: '54', peripheral: CMP0, signal: 'IN, 3', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/I2S0_RX_BCLK/FTM2_FLT0}
  - {pin_num: '55', peripheral: I2C1, signal: SCL, pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/I2S0_RX_FS, pull_select: up}
  - {pin_num: '56', peripheral: I2C1, signal: SDA, pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA, pull_select: up}
  - {pin_num: '57', peripheral: SPI0, signal: PCS0_SS, pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/LPUART0_RTS_b}
  - {pin_num: '58', peripheral: SPI0, signal: SCK, pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/LPUART0_CTS_b}
  - {pin_num: '59', peripheral: SPI0, signal: SOUT, pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/LPUART0_RX/I2C0_SCL}
  - {pin_num: '60', peripheral: SPI0, signal: SIN, pin_signal: PTD3/SPI0_SIN/UART2_TX/LPUART0_TX/I2C0_SDA}
  - {pin_num: '61', peripheral: GPIOD, signal: 'GPIO, 4', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/EWM_IN/SPI1_PCS0, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '62', peripheral: GPIOD, signal: 'GPIO, 5', pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/FTM0_CH5/EWM_OUT_b/SPI1_SCK, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '63', peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FTM0_FLT0/SPI1_SOUT, direction: INPUT, pull_select: up,
    pull_enable: enable}
  - {pin_num: '64', peripheral: GPIOD, signal: 'GPIO, 7', pin_signal: PTD7/UART0_TX/FTM0_CH7/FTM0_FLT1/SPI1_SIN, direction: INPUT, pull_select: up, pull_enable: enable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortD);                           /* Port D Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortE);                           /* Port E Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTA, PIN12_IDX, kPORT_MuxAlt3);           /* PORTA12 (pin 28) is configured as FTM1_CH0 */
  PORTA->PCR[12] = ((PORTA->PCR[12] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTA, PIN13_IDX, kPORT_MuxAlt3);           /* PORTA13 (pin 29) is configured as FTM1_CH1 */
  PORTA->PCR[13] = ((PORTA->PCR[13] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTA, PIN18_IDX, kPORT_MuxAsGpio);         /* PORTA18 (pin 32) is configured as PTA18 */
  PORTA->PCR[18] = ((PORTA->PCR[18] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTA, PIN19_IDX, kPORT_MuxAsGpio);         /* PORTA19 (pin 33) is configured as PTA19 */
  PORTA->PCR[19] = ((PORTA->PCR[19] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTA, PIN5_IDX, kPORT_MuxAlt3);            /* PORTA5 (pin 27) is configured as FTM0_CH2 */
  PORTA->PCR[5] = ((PORTA->PCR[5] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                             	 /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTB, PIN0_IDX, kPORT_PinDisabledOrAnalog); /* PORTB0 (pin 35) is configured as ADC1_SE8 */
  PORT_SetPinMux(PORTB, PIN1_IDX, kPORT_PinDisabledOrAnalog); /* PORTB1 (pin 36) is configured as ADC1_SE9 */
  PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAsGpio);         /* PORTB16 (pin 39) is configured as PTB16 */
  PORTB->PCR[16] = ((PORTB->PCR[16] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAsGpio);         /* PORTB17 (pin 40) is configured as PTB17 */
  PORTB->PCR[17] = ((PORTB->PCR[17] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTB, PIN18_IDX, kPORT_MuxAsGpio);         /* PORTB18 (pin 41) is configured as PTB18 */
  PORTB->PCR[18] = ((PORTB->PCR[18] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTB, PIN19_IDX, kPORT_MuxAsGpio);         /* PORTB19 (pin 42) is configured as PTB19 */
  PORTB->PCR[19] = ((PORTB->PCR[19] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTB, PIN2_IDX, kPORT_PinDisabledOrAnalog); /* PORTB2 (pin 37) is configured as ADC0_SE12 */
  PORT_SetPinMux(PORTB, PIN3_IDX, kPORT_PinDisabledOrAnalog); /* PORTB3 (pin 38) is configured as ADC0_SE13 */
  PORT_SetPinMux(PORTC, PIN0_IDX, kPORT_PinDisabledOrAnalog); /* PORTC0 (pin 43) is configured as ADC0_SE14 */
  PORT_SetPinMux(PORTC, PIN1_IDX, kPORT_MuxAlt3);            /* PORTC1 (pin 44) is configured as UART1_RTS_b */
  PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_MuxAlt2);           /* PORTC10 (pin 55) is configured as I2C1_SCL */
  PORTC->PCR[10] = ((PORTC->PCR[10] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
    );
  PORT_SetPinMux(PORTC, PIN11_IDX, kPORT_MuxAlt2);           /* PORTC11 (pin 56) is configured as I2C1_SDA */
  PORTC->PCR[11] = ((PORTC->PCR[11] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP) | PORT_PCR_ODE(true)          /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
    );
  PORT_SetPinMux(PORTC, PIN2_IDX, kPORT_MuxAsGpio);          /* PORTC2 (pin 45) is configured as PTC2 */
  PORTC->PCR[2] = ((PORTC->PCR[2] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTC, PIN3_IDX, kPORT_MuxAsGpio);          /* PORTC3 (pin 46) is configured as PTC3 */
  PORTC->PCR[3] = ((PORTC->PCR[3] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTC, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTC4 (pin 49) is configured as PTC4 */
  PORTC->PCR[4] = ((PORTC->PCR[4] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
    );
  PORT_SetPinMux(PORTC, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTC5 (pin 50) is configured as PTC5 */
  PORTC->PCR[5] = ((PORTC->PCR[5] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTC, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTC6 (pin 51) is configured as PTC6 */
  PORTC->PCR[6] = ((PORTC->PCR[6] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTC, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTC7 (pin 52) is configured as PTC7 */
  PORTC->PCR[7] = ((PORTC->PCR[7] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTC, PIN8_IDX, kPORT_PinDisabledOrAnalog); /* PORTC8 (pin 53) is configured as ADC1_SE4b, CMP0_IN2 */
  PORT_SetPinMux(PORTC, PIN9_IDX, kPORT_PinDisabledOrAnalog); /* PORTC9 (pin 54) is configured as CMP0_IN3, ADC1_SE5b */
  PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_MuxAlt2);            /* PORTD0 (pin 57) is configured as SPI0_PCS0 */
  PORT_SetPinMux(PORTD, PIN1_IDX, kPORT_MuxAlt2);            /* PORTD1 (pin 58) is configured as SPI0_SCK */
  PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_MuxAlt2);            /* PORTD2 (pin 59) is configured as SPI0_SOUT */
  PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAlt2);            /* PORTD3 (pin 60) is configured as SPI0_SIN */
  PORT_SetPinMux(PORTD, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTD4 (pin 61) is configured as PTD4 */
  PORTD->PCR[4] = ((PORTD->PCR[4] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTD, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTD5 (pin 62) is configured as PTD5 */
  PORTD->PCR[5] = ((PORTD->PCR[5] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTD, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTD6 (pin 63) is configured as PTD6 */
  PORTD->PCR[6] = ((PORTD->PCR[6] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
    );
  PORT_SetPinMux(PORTD, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTD7 (pin 64) is configured as PTD7 */
  PORTD->PCR[7] = ((PORTD->PCR[7] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
    );
  PORT_SetPinMux(PORTE, PIN0_IDX, kPORT_MuxAlt3);            /* PORTE0 (pin 1) is configured as UART1_TX */
  PORT_SetPinMux(PORTE, PIN1_IDX, kPORT_MuxAlt3);            /* PORTE1 (pin 2) is configured as UART1_RX */
  SIM->SOPT4 = ((SIM->SOPT4 &
    (~(SIM_SOPT4_FTM1CH0SRC_MASK)))                          /* Mask bits to zero which are setting */
      | SIM_SOPT4_FTM1CH0SRC(SOPT4_FTM1CH0SRC_FTM)           /* FTM1 channel 0 input capture source select: FTM1_CH0 signal */
    );
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART1TXSRC_MASK)))                          /* Mask bits to zero which are setting */
      | SIM_SOPT5_UART1TXSRC(SOPT5_UART1TXSRC_UART_TX)       /* UART 1 transmit data source select: UART1_TX pin */
    );
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
