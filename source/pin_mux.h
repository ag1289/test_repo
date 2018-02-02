#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* PORTA5 (number 27), PWM_PV */
#define PWM_PV_PERIPHERAL                                    FTM0   /*!< Device name: FTM0 */
#define PWM_PV_SIGNAL                                          CH   /*!< FTM0 signal: CH */
#define PWM_PV_CHANNEL                                          2   /*!< FTM0 channel: 2 */
#define PWM_PV_PIN_NAME                                  FTM0_CH2   /*!< Pin name */
#define PWM_PV_LABEL                                     "PWM_PV"   /*!< Label */
#define PWM_PV_NAME                                      "PWM_PV"   /*!< Identifier name */
#define PWM_PV_DIRECTION                 kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTA12 (number 28), IN_BRIDGE_1 */
#define IN_BRIDGE_1_PERIPHERAL                               FTM1   /*!< Device name: FTM1 */
#define IN_BRIDGE_1_SIGNAL                                     CH   /*!< FTM1 signal: CH */
#define IN_BRIDGE_1_CHANNEL                                     0   /*!< FTM1 channel: 0 */
#define IN_BRIDGE_1_PIN_NAME                             FTM1_CH0   /*!< Pin name */
#define IN_BRIDGE_1_LABEL                           "IN_BRIDGE_1"   /*!< Label */
#define IN_BRIDGE_1_NAME                            "IN_BRIDGE_1"   /*!< Identifier name */
#define IN_BRIDGE_1_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTA13 (number 29), IN_BRIDGE_2 */
#define IN_BRIDGE_2_PERIPHERAL                               FTM1   /*!< Device name: FTM1 */
#define IN_BRIDGE_2_SIGNAL                                     CH   /*!< FTM1 signal: CH */
#define IN_BRIDGE_2_CHANNEL                                     1   /*!< FTM1 channel: 1 */
#define IN_BRIDGE_2_PIN_NAME                             FTM1_CH1   /*!< Pin name */
#define IN_BRIDGE_2_LABEL                           "IN_BRIDGE_2"   /*!< Label */
#define IN_BRIDGE_2_NAME                            "IN_BRIDGE_2"   /*!< Identifier name */
#define IN_BRIDGE_2_DIRECTION            kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTA18 (number 32), INH_BRIDGE1 */
#define INH_BRIDGE1_GPIO                                    GPIOA   /*!< GPIO device name: GPIOA */
#define INH_BRIDGE1_PORT                                    PORTA   /*!< PORT device name: PORTA */
#define INH_BRIDGE1_GPIO_PIN                                  18U   /*!< PORTA pin index: 18 */
#define INH_BRIDGE1_PIN_NAME                                PTA18   /*!< Pin name */
#define INH_BRIDGE1_LABEL                           "INH_BRIDGE1"   /*!< Label */
#define INH_BRIDGE1_NAME                            "INH_BRIDGE1"   /*!< Identifier name */
#define INH_BRIDGE1_DIRECTION             kPIN_MUX_DirectionOutput  /*!< Direction */

/* PORTA19 (number 33), AXIS_OUT1 */
#define AXIS_OUT1_GPIO                                      GPIOA   /*!< GPIO device name: GPIOA */
#define AXIS_OUT1_PORT                                      PORTA   /*!< PORT device name: PORTA */
#define AXIS_OUT1_GPIO_PIN                                    19U   /*!< PORTA pin index: 19 */
#define AXIS_OUT1_PIN_NAME                                  PTA19   /*!< Pin name */
#define AXIS_OUT1_LABEL                               "AXIS_OUT1"   /*!< Label */
#define AXIS_OUT1_NAME                                "AXIS_OUT1"   /*!< Identifier name */
#define AXIS_OUT1_DIRECTION              kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTB16 (number 39), KEYB0 */
#define KEYB0_GPIO                                          GPIOB   /*!< GPIO device name: GPIOB */
#define KEYB0_PORT                                          PORTB   /*!< PORT device name: PORTB */
#define KEYB0_GPIO_PIN                                        16U   /*!< PORTB pin index: 16 */
#define KEYB0_PIN_NAME                                      PTB16   /*!< Pin name */
#define KEYB0_LABEL                                       "KEYB0"   /*!< Label */
#define KEYB0_NAME                                        "KEYB0"   /*!< Identifier name */
#define KEYB0_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTB17 (number 40), KEYB1 */
#define KEYB1_GPIO                                          GPIOB   /*!< GPIO device name: GPIOB */
#define KEYB1_PORT                                          PORTB   /*!< PORT device name: PORTB */
#define KEYB1_GPIO_PIN                                        17U   /*!< PORTB pin index: 17 */
#define KEYB1_PIN_NAME                                      PTB17   /*!< Pin name */
#define KEYB1_LABEL                                       "KEYB1"   /*!< Label */
#define KEYB1_NAME                                        "KEYB1"   /*!< Identifier name */
#define KEYB1_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTB18 (number 41), KEYB2 */
#define KEYB2_GPIO                                          GPIOB   /*!< GPIO device name: GPIOB */
#define KEYB2_PORT                                          PORTB   /*!< PORT device name: PORTB */
#define KEYB2_GPIO_PIN                                        18U   /*!< PORTB pin index: 18 */
#define KEYB2_PIN_NAME                                      PTB18   /*!< Pin name */
#define KEYB2_LABEL                                       "KEYB2"   /*!< Label */
#define KEYB2_NAME                                        "KEYB2"   /*!< Identifier name */
#define KEYB2_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTB19 (number 42), KEYB3 */
#define KEYB3_GPIO                                          GPIOB   /*!< GPIO device name: GPIOB */
#define KEYB3_PORT                                          PORTB   /*!< PORT device name: PORTB */
#define KEYB3_GPIO_PIN                                        19U   /*!< PORTB pin index: 19 */
#define KEYB3_PIN_NAME                                      PTB19   /*!< Pin name */
#define KEYB3_LABEL                                       "KEYB3"   /*!< Label */
#define KEYB3_NAME                                        "KEYB3"   /*!< Identifier name */
#define KEYB3_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTC1 (number 45), E_RS485 R/W */
#define RTC_RST_GPIO                                        GPIOC   /*!< GPIO device name: GPIOC */
#define RTC_RST_PORT                                        PORTC   /*!< PORT device name: PORTC */
#define RTC_RST_GPIO_PIN                                       2U   /*!< PORTC pin index: 2 */
#define RTC_RST_PIN_NAME                                     PTC2   /*!< Pin name */
#define RTC_RST_LABEL                                   "RTC_RST"   /*!< Label */
#define RTC_RST_NAME                                    "RTC_RST"   /*!< Identifier name */
#define RTC_RST_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC2 (number 46), DIN0 */
#define DIN0_GPIO                                        GPIOC   /*!< GPIO device name: GPIOC */
#define DIN0_PORT                                        PORTC   /*!< PORT device name: PORTC */
#define DIN0_GPIO_PIN                                       2U   /*!< PORTC pin index: 2 */
#define DIN0_PIN_NAME                                     PTC2   /*!< Pin name */
#define DIN0_LABEL                                   "DIN0"   /*!< Label */
#define DIN0_NAME                                    "DIN0"   /*!< Identifier name */
#define DIN0_DIRECTION                 kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTC3 (number 46), DIN1 */
#define DIN1_GPIO                                        GPIOC   /*!< GPIO device name: GPIOC */
#define DIN1_PORT                                        PORTC   /*!< PORT device name: PORTC */
#define DIN1_GPIO_PIN                                       3U   /*!< PORTC pin index: 3 */
#define DIN1_PIN_NAME                                     PTC3   /*!< Pin name */
#define DIN1_LABEL                                   "DIN1"   /*!< Label */
#define DIN1_NAME                                    "DIN1"   /*!< Identifier name */
#define DIN1_DIRECTION                 kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTC4 (number 49), LED STATUS */
#define STATUSLED_GPIO                                        GPIOC   /*!< GPIO device name: GPIOC */
#define STATUSLED_PORT                                        PORTC   /*!< PORT device name: PORTC */
#define STATUSLED_GPIO_PIN                                       4U   /*!< PORTC pin index: 4 */
#define STATUSLED_PIN_NAME                                     PTC4   /*!< Pin name */
#define STATUSLED_LABEL                                   "LED_STATUS"   /*!< Label */
#define STATUSLED_NAME                                    "LED_STATUS"   /*!< Identifier name */
#define STATUSLED_DIRECTION                 kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC5 (number 50), MEMS_INT */
#define MEMS_INT_GPIO                                       GPIOC   /*!< GPIO device name: GPIOC */
#define MEMS_INT_PORT                                       PORTC   /*!< PORT device name: PORTC */
#define MEMS_INT_GPIO_PIN                                      5U   /*!< PORTC pin index: 5 */
#define MEMS_INT_PIN_NAME                                    PTC5   /*!< Pin name */
#define MEMS_INT_LABEL                                 "MEMS_INT"   /*!< Label */
#define MEMS_INT_NAME                                  "MEMS_INT"   /*!< Identifier name */
#define MEMS_INT_DIRECTION                kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTC6 (number 51), RFM_RST */
#define RFM_RST_GPIO                                        GPIOC   /*!< GPIO device name: GPIOC */
#define RFM_RST_PORT                                        PORTC   /*!< PORT device name: PORTC */
#define RFM_RST_GPIO_PIN                                       6U   /*!< PORTC pin index: 6 */
#define RFM_RST_PIN_NAME                                     PTC6   /*!< Pin name */
#define RFM_RST_LABEL                                   "RFM_RST"   /*!< Label */
#define RFM_RST_NAME                                    "RFM_RST"   /*!< Identifier name */
#define RFM_RST_DIRECTION                kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC7 (number 52), AXIS_OUT2 */
#define AXIS_OUT2_GPIO                                      GPIOC   /*!< GPIO device name: GPIOC */
#define AXIS_OUT2_PORT                                      PORTC   /*!< PORT device name: PORTC */
#define AXIS_OUT2_GPIO_PIN                                     7U   /*!< PORTC pin index: 7 */
#define AXIS_OUT2_PIN_NAME                                   PTC7   /*!< Pin name */
#define AXIS_OUT2_LABEL                               "AXIS_OUT2"   /*!< Label */
#define AXIS_OUT2_NAME                                "AXIS_OUT2"   /*!< Identifier name */
#define AXIS_OUT2_DIRECTION              kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC10(number 53), I2C_SCK */
#define I2C1_SCL_GPIO                                      GPIOC   /*!< GPIO device name: GPIOC */
#define I2C1_SCL_PORT                                      PORTC   /*!< PORT device name: PORTC */
#define I2C1_SCL_GPIO_PIN                                    10U   /*!< PORTC pin index: 7 */
#define I2C1_SCL_PIN_NAME                                  PTC10   /*!< Pin name */
#define I2C1_SCL_LABEL                               "I2C1_SCL"   /*!< Label */
#define I2C1_SCL_NAME                                "I2C1_SCL"   /*!< Identifier name */
#define I2C1_SCL_DIRECTION              kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD4 (number 61), KEYB4 */
#define KEYB4_GPIO                                          GPIOD   /*!< GPIO device name: GPIOD */
#define KEYB4_PORT                                          PORTD   /*!< PORT device name: PORTD */
#define KEYB4_GPIO_PIN                                         4U   /*!< PORTD pin index: 4 */
#define KEYB4_PIN_NAME                                       PTD4   /*!< Pin name */
#define KEYB4_LABEL                                       "KEYB4"   /*!< Label */
#define KEYB4_NAME                                        "KEYB4"   /*!< Identifier name */
#define KEYB4_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD5 (number 62), KEYB5 */
#define KEYB5_GPIO                                          GPIOD   /*!< GPIO device name: GPIOD */
#define KEYB5_PORT                                          PORTD   /*!< PORT device name: PORTD */
#define KEYB5_GPIO_PIN                                         5U   /*!< PORTD pin index: 5 */
#define KEYB5_PIN_NAME                                       PTD5   /*!< Pin name */
#define KEYB5_LABEL                                       "KEYB5"   /*!< Label */
#define KEYB5_NAME                                        "KEYB5"   /*!< Identifier name */
#define KEYB5_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD6 (number 63), KEYB6 */
#define KEYB6_GPIO                                          GPIOD   /*!< GPIO device name: GPIOD */
#define KEYB6_PORT                                          PORTD   /*!< PORT device name: PORTD */
#define KEYB6_GPIO_PIN                                         6U   /*!< PORTD pin index: 6 */
#define KEYB6_PIN_NAME                                       PTD6   /*!< Pin name */
#define KEYB6_LABEL                                       "KEYB6"   /*!< Label */
#define KEYB6_NAME                                        "KEYB6"   /*!< Identifier name */
#define KEYB6_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTD7 (number 64), KEYB7 */
#define KEYB7_GPIO                                          GPIOD   /*!< GPIO device name: GPIOD */
#define KEYB7_PORT                                          PORTD   /*!< PORT device name: PORTD */
#define KEYB7_GPIO_PIN                                         7U   /*!< PORTD pin index: 7 */
#define KEYB7_PIN_NAME                                       PTD7   /*!< Pin name */
#define KEYB7_LABEL                                       "KEYB7"   /*!< Label */
#define KEYB7_NAME                                        "KEYB7"   /*!< Identifier name */
#define KEYB7_DIRECTION                   kPIN_MUX_DirectionInput   /*!< Direction */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
