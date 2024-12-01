/*
 * HEE_board.h
 *
 *  Created on: Jul 27, 2024
 *      Author: duong
 */

#ifndef BOARD_H_
#define BOARD_H_

/*********************H BRIDGE******************/
#define H_BRIDGE_SD0_3_HANDLE   TIM1

#define H_BRIDGE_SD0_CHANNEL    LL_TIM_CHANNEL_CH1

#define H_BRIDGE_SD1_CHANNEL    LL_TIM_CHANNEL_CH2

#define H_BRIDGE_SD2_CHANNEL    LL_TIM_CHANNEL_CH3

#define H_BRIDGE_SD3_CHANNEL    LL_TIM_CHANNEL_CH4

#define H_BRIDGE_SD4_7_HANDLE   TIM8

#define H_BRIDGE_SD4_CHANNEL    LL_TIM_CHANNEL_CH4

#define H_BRIDGE_SD5_CHANNEL    LL_TIM_CHANNEL_CH3

#define H_BRIDGE_SD6_CHANNEL    LL_TIM_CHANNEL_CH2

#define H_BRIDGE_SD7_CHANNEL    LL_TIM_CHANNEL_CH1

//________________________________________________
#define H_BRIDGE_HIN0_7_PORT    GPIOB

#define H_BRIDGE_HIN0_PIN       LL_GPIO_PIN_1

#define H_BRIDGE_HIN1_PIN       LL_GPIO_PIN_2

#define H_BRIDGE_HIN2_PIN       LL_GPIO_PIN_10

#define H_BRIDGE_HIN3_PIN       LL_GPIO_PIN_11

#define H_BRIDGE_HIN4_PIN       LL_GPIO_PIN_15

#define H_BRIDGE_HIN5_PIN       LL_GPIO_PIN_14

#define H_BRIDGE_HIN6_PIN       LL_GPIO_PIN_13

#define H_BRIDGE_HIN7_PIN       LL_GPIO_PIN_12

/***********************************************/

/*****************Voltage Switching*************/
#define V_SWITCH_LIN1_HANDLE    TIM2
#define V_SWITCH_LIN1_CHANNEL   LL_TIM_CHANNEL_CH2

#define V_SWITCH_HIN1_PORT      GPIOA
#define V_SWITCH_HIN1_PIN       LL_GPIO_PIN_3

#define V_SWITCH_LIN2_HANDLE    TIM2
#define V_SWITCH_LIN2_CHANNEL   LL_TIM_CHANNEL_CH3

#define V_SWITCH_HIN2_PORT      GPIOA
#define V_SWITCH_HIN2_PIN       LL_GPIO_PIN_0
/***********************************************/

/**************CURRENT MONITOR*****************/
#define ADC_I_SENSE_HANDLE      ADC1
#define ADC_I_SENSE_CHANNEL     LL_ADC_CHANNEL_10
#define ADC_I_SENSE_IRQ         ADC_IRQn

//#define I_SENSE_TIMER_HANDLE    TIM3

//#define OCP_ALERT_PORT          GPIOC
//#define OCP_ALERT_PIN           LL_GPIO_PIN_13

//#define I_SENSE_RESET_PORT      GPIOA
//#define I_SENSE_RESET_PIN       LL_GPIO_PIN_15
#define ADC_CHANNEL_COUNT		1
/***********************************************/

/*********************UART**********************/
#define RS232_UART_HANDLE       UART4
#define RS232_UART_IRQ          UART4_IRQn

#define GPC_UART_HANDLE         USART1
#define GPC_UART_IRQ            USART1_IRQn
/***********************************************/

/**********************I2C SENSOR***************/
#define SENSOR_COUNT            2

#define I2C_HANDLE              I2C1
/***********************************************/

/*******************DEBUG LED*******************/
#define DEBUG_LED_PORT          GPIOC
#define DEBUG_LED_PIN           LL_GPIO_PIN_4
/***********************************************/

#define GPP_TX_SIZE			64
#define	GPP_RX_SIZE			64
#define FSP_BUF_LEN			64

#endif /* BOARD_H_ */
