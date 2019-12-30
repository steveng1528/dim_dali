/**
  ******************************************************************************
  * @file    platform_config.h 
  * @author  Thai.Pham
  * @version V1.0
  * @date    October 01, 2009
  * @brief   specific configuration file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
#include "stm32f10x_exti.h"

/* Global configuration -------------------------------------------------------------*/

/* Global define -------------------------------------------------------------*/
typedef enum { TRUE = 1, FALSE = 0 } bool ;
typedef bool BOOL;   /* IGNORESTYLE */

/* Define the hardware depending on the used board  --------------------------*/
#ifdef  HAS_ROUTER
    #ifdef OLD_ROUTER_BOARD
        #define RESET_GPIO_PIN               GPIO_Pin_5
        #define RESET_GPIO_PORT              GPIOC
        #define RESET_GPIO_CLK               RCC_APB2Periph_GPIOC
        
        #define CTS_GPIO_PIN                 GPIO_Pin_10
        #define CTS_GPIO_PORT                GPIOC
        #define CTS_GPIO_CLK                 RCC_APB2Periph_GPIOC
        #define CTS_GPIO_EXTI_LINE           EXTI_Line10
        #define CTS_GPIO_PORT_SOURCE         GPIO_PortSourceGPIOC
        #define CTS_GPIO_PIN_SOURCE          GPIO_PinSource10
        #define CTS_GPIO_IRQn                EXTI15_10_IRQn
    #else
        #define RESET_GPIO_PIN               GPIO_Pin_5
        #define RESET_GPIO_PORT              GPIOA
        #define RESET_GPIO_CLK               RCC_APB2Periph_GPIOA

        #define CTS_GPIO_PIN                 GPIO_Pin_4
        #define CTS_GPIO_PORT                GPIOC
        #define CTS_GPIO_CLK                 RCC_APB2Periph_GPIOC
        #define CTS_GPIO_EXTI_LINE           EXTI_Line4
        #define CTS_GPIO_PORT_SOURCE         GPIO_PortSourceGPIOC
        #define CTS_GPIO_PIN_SOURCE          GPIO_PinSource4
        #define CTS_GPIO_IRQn                EXTI4_IRQn
    #endif
        #define RTS_GPIO_PIN                 GPIO_Pin_5
        #define RTS_GPIO_PORT                GPIOC
        #define RTS_GPIO_CLK                 RCC_APB2Periph_GPIOC

        #define CONFIG_GPIO_PIN              GPIO_Pin_0
        #define CONFIG_GPIO_PORT             GPIOB
        #define CONFIG_GPIO_CLK              RCC_APB2Periph_GPIOB
        
        #define LED0_GPIO_PIN                GPIO_Pin_5
        #define LED0_GPIO_PORT               GPIOB
        #define LED0_GPIO_CLK                RCC_APB2Periph_GPIOB

        #define LED1_GPIO_PIN                GPIO_Pin_6
        #define LED1_GPIO_PORT               GPIOB
        #define LED1_GPIO_CLK                RCC_APB2Periph_GPIOB

        #define LED2_GPIO_PIN                GPIO_Pin_7
        #define LED2_GPIO_PORT               GPIOB
        #define LED2_GPIO_CLK                RCC_APB2Periph_GPIOB
        

#else
    #define RESET_GPIO_PIN               GPIO_Pin_1
    #define RESET_GPIO_PORT              GPIOB
    #define RESET_GPIO_CLK               RCC_APB2Periph_GPIOB

    #define CONFIG_GPIO_PIN              GPIO_Pin_1
    #define CONFIG_GPIO_PORT             GPIOA
    #define CONFIG_GPIO_CLK              RCC_APB2Periph_GPIOA

    #define LED0_GPIO_PIN                GPIO_Pin_14
    #define LED0_GPIO_PORT               GPIOB
    #define LED0_GPIO_CLK                RCC_APB2Periph_GPIOB

    #define LED1_GPIO_PIN                GPIO_Pin_15
    #define LED1_GPIO_PORT               GPIOB
    #define LED1_GPIO_CLK                RCC_APB2Periph_GPIOB

    #define LED2_GPIO_PIN                GPIO_Pin_0
    #define LED2_GPIO_PORT               GPIOB
    #define LED2_GPIO_CLK                RCC_APB2Periph_GPIOB

    #ifdef  OLD_GATEWAY_BOARD
    #define CTS_GPIO_PIN                 GPIO_Pin_5
    #define CTS_GPIO_PORT                GPIOC
    #define CTS_GPIO_CLK                 RCC_APB2Periph_GPIOC
    #define CTS_GPIO_EXTI_LINE           EXTI_Line5
    #define CTS_GPIO_PORT_SOURCE         GPIO_PortSourceGPIOC
    #define CTS_GPIO_PIN_SOURCE          GPIO_PinSource5
    #define CTS_GPIO_IRQn                EXTI9_5_IRQn
    #else
    #define CTS_GPIO_PIN                 GPIO_Pin_5
    #define CTS_GPIO_PORT                GPIOA
    #define CTS_GPIO_CLK                 RCC_APB2Periph_GPIOA
    #define CTS_GPIO_EXTI_LINE           EXTI_Line5
    #define CTS_GPIO_PORT_SOURCE         GPIO_PortSourceGPIOA
    #define CTS_GPIO_PIN_SOURCE          GPIO_PinSource5
    #define CTS_GPIO_IRQn                EXTI9_5_IRQn
    #endif
    
    #define RTS_GPIO_PIN                 GPIO_Pin_4
    #define RTS_GPIO_PORT                GPIOA
    #define RTS_GPIO_CLK                 RCC_APB2Periph_GPIOA
#endif

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RF_RX_LED_Blinking( void );
void RF_TX_LED_Blinking( void );
void RF_init_LED_Stop_Blink( void );
void RF_init_LED_Blink_cont( void );
void RF_LED_Blinking( void );
#define RF_LED_Blinking()   RF_TX_LED_Blinking()
void delay_ms(unsigned int x);
unsigned long X_GetTimerTickIn_ms(void);
unsigned char  sm_Ascii2Hex(unsigned char x);
void  SM_Hex2Ascii(unsigned char x, unsigned char* dst);

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2014 Thai.Pham *****END OF FILE****/
